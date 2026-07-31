#!/usr/bin/env python3
"""
HLK-LD2417 Vehicle Distance / Lane / Speed Monitor
====================================================

Reads target reports from a Hi-Link HLK-LD2417 (24GHz vehicle-status
radar, same firmware family as the LD2451) over UART, and for every
detected vehicle reports:

    - distance (m)
    - speed (km/h)
    - lane number (derived from lateral position)

IMPORTANT - READ BEFORE USING
------------------------------
Hi-Link revises the exact byte layout of the report frame across
firmware/module revisions, and I was not able to pull a byte-verified
copy of the LD2417's datasheet. The FRAME_* constants and the
`parse_target()` field offsets below are my best-effort based on the
publicly documented Hi-Link vehicle-radar family (LD2451) and general
Hi-Link "LD" report structure. **Before trusting the output, confirm
the following against the PDF datasheet that shipped with your module
(or the current one from hlktech.net / manuals.plus):**

    1. FRAME_HEADER / FRAME_TAIL byte sequences
    2. Whether fields are little-endian or big-endian
    3. The exact byte offset/width of: target count, distance,
       speed, angle, direction flag, and (if present) a raw lane index
    4. Whether angle is signed degrees, or already given in radians

Run with `--debug-raw` first. It prints the raw hex of every frame
the module sends. Compare that against your datasheet's frame diagram
and adjust `FRAME_HEADER`, `FRAME_TAIL`, and `parse_target()` to match
-- most Hi-Link "LD" radars use a 4-byte header, a 2-byte little-endian
length field, N x fixed-size target records, then a 4-byte tail, but
the exact byte values differ by module.

Dependencies
------------
    pip install pyserial --break-system-packages

Usage
-----
    python3 ld2417_traffic_monitor.py --port /dev/ttyUSB0
    python3 ld2417_traffic_monitor.py --port COM5 --debug-raw
    python3 ld2417_traffic_monitor.py --port /dev/ttyUSB0 --csv log.csv --lanes 3 --lane-width 3.5
"""

import argparse
import csv
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from math import radians, sin
from typing import Optional

try:
    import serial
except ImportError:
    print("Missing dependency. Install with:\n"
          "    pip install pyserial --break-system-packages")
    sys.exit(1)


# ----------------------------------------------------------------------
# CONFIG - verify these against your datasheet (see module docstring)
# ----------------------------------------------------------------------

BAUD_RATE = 256000          # Hi-Link "LD" series default UART baud rate
FRAME_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])   # <-- VERIFY
FRAME_TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])     # <-- VERIFY
MAX_TARGETS_PER_FRAME = 8   # safety cap while scanning for frames

# Byte layout of a single target record, VERIFY against datasheet.
# Assumed record (little-endian int16 fields, matching the LD2450/2451
# target-record convention: distance, speed, and lateral/angle info):
#   distance_cm : int16
#   speed_kmh   : int16  (signed; negative = approaching, positive = receding
#                          -- OR the reverse; verify polarity on your unit)
#   angle_deg   : int16  (signed, degrees from boresight, 0 = straight ahead)
TARGET_RECORD_STRUCT = struct.Struct("<hhh")   # distance, speed, angle
TARGET_RECORD_SIZE = TARGET_RECORD_STRUCT.size

# ----------------------------------------------------------------------


@dataclass
class Target:
    distance_m: float
    speed_kmh: float
    angle_deg: float
    lateral_m: float = field(init=False)
    lane: Optional[int] = field(init=False, default=None)

    def __post_init__(self):
        # Lateral offset from boresight, used for lane assignment.
        self.lateral_m = self.distance_m * sin(radians(self.angle_deg))


class LaneMapper:
    """
    Buckets targets into lanes based on lateral offset from the radar's
    boresight. Assumes the radar is mounted so its boresight points
    across the lanes (e.g. roadside pole aimed across the carriageway,
    or overhead looking straight down the road with angle representing
    cross-road position).

    lane_count : total number of lanes to report
    lane_width : width of each lane in meters
    center_offset : lateral distance (m) from the radar to the edge of
                    lane 1, i.e. where lane numbering starts. Adjust
                    this to match your physical mounting.
    """

    def __init__(self, lane_count: int, lane_width: float, center_offset: float = 0.0):
        self.lane_count = lane_count
        self.lane_width = lane_width
        self.center_offset = center_offset

    def assign(self, lateral_m: float) -> Optional[int]:
        adjusted = lateral_m - self.center_offset
        lane_index = int(adjusted // self.lane_width)
        if 0 <= lane_index < self.lane_count:
            return lane_index + 1  # 1-indexed lanes for humans
        return None  # outside configured lane span


class SpeedEstimator:
    """
    Fallback speed estimator via distance differentiation, used only if
    a frame's speed field looks implausible (e.g. sensor reports 0 for
    every target, which some firmware does when engineering mode is off).
    Keeps a short history per lane to smooth out noise.
    """

    def __init__(self, history_len: int = 5):
        self.history = {}  # lane -> deque[(timestamp, distance_m)]
        self.history_len = history_len

    def estimate(self, lane: int, distance_m: float, now: float) -> Optional[float]:
        if lane not in self.history:
            self.history[lane] = deque(maxlen=self.history_len)
        hist = self.history[lane]
        hist.append((now, distance_m))
        if len(hist) < 2:
            return None
        (t0, d0), (t1, d1) = hist[0], hist[-1]
        dt = t1 - t0
        if dt <= 0:
            return None
        speed_mps = (d0 - d1) / dt  # positive = approaching
        return speed_mps * 3.6  # -> km/h


class FrameReader:
    """
    Buffers incoming serial bytes and yields complete frames delimited
    by FRAME_HEADER ... FRAME_TAIL.
    """

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = bytearray()

    def read_frames(self):
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if chunk:
            self.buf.extend(chunk)

        while True:
            start = self.buf.find(FRAME_HEADER)
            if start == -1:
                # No header yet; drop stale bytes to avoid unbounded growth
                if len(self.buf) > 4096:
                    self.buf.clear()
                return
            end = self.buf.find(FRAME_TAIL, start + len(FRAME_HEADER))
            if end == -1:
                # Header found but tail not arrived yet; wait for more data
                if start > 0:
                    del self.buf[:start]  # discard junk before header
                return
            payload = bytes(self.buf[start + len(FRAME_HEADER):end])
            del self.buf[:end + len(FRAME_TAIL)]
            yield payload


def parse_targets(payload: bytes):
    """
    Splits a frame payload into individual target records and decodes
    each one. Returns a list of Target objects (distance in meters,
    speed in km/h as reported by the module, angle in degrees).

    VERIFY: this assumes the payload is just back-to-back fixed-size
    target records with no extra header byte for target count. If your
    datasheet shows a 1-2 byte "target count" field before the records,
    slice it off here first, e.g.:
        target_count = payload[0]
        payload = payload[1:]
    """
    targets = []
    n_records = min(len(payload) // TARGET_RECORD_SIZE, MAX_TARGETS_PER_FRAME)
    for i in range(n_records):
        chunk = payload[i * TARGET_RECORD_SIZE:(i + 1) * TARGET_RECORD_SIZE]
        raw_distance, raw_speed, raw_angle = TARGET_RECORD_STRUCT.unpack(chunk)

        # A record of all zeros usually means "empty slot" on Hi-Link
        # LD-series modules -- skip it.
        if raw_distance == 0 and raw_speed == 0 and raw_angle == 0:
            continue

        distance_m = raw_distance / 100.0   # assumed cm -> m, VERIFY
        speed_kmh = float(raw_speed)        # assumed already km/h, VERIFY
        angle_deg = float(raw_angle)

        targets.append(Target(distance_m=distance_m, speed_kmh=speed_kmh, angle_deg=angle_deg))
    return targets


def main():
    parser = argparse.ArgumentParser(description="HLK-LD2417 distance/lane/speed monitor")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="UART baud rate")
    parser.add_argument("--lanes", type=int, default=3, help="Number of lanes to detect")
    parser.add_argument("--lane-width", type=float, default=3.5, help="Lane width in meters")
    parser.add_argument("--center-offset", type=float, default=-5.25,
                         help="Lateral distance (m) from radar boresight to start of lane 1. "
                              "Negative if lane 1 is to the left of boresight.")
    parser.add_argument("--csv", type=str, default=None, help="Optional path to log detections as CSV")
    parser.add_argument("--use-fallback-speed", action="store_true",
                         help="Estimate speed from distance-over-time instead of the module's speed field")
    parser.add_argument("--debug-raw", action="store_true",
                         help="Print raw hex of every frame instead of parsed output (use this first!)")
    args = parser.parse_args()

    lane_mapper = LaneMapper(args.lanes, args.lane_width, args.center_offset)
    speed_estimator = SpeedEstimator()

    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(["timestamp", "lane", "distance_m", "speed_kmh", "angle_deg"])

    print(f"Opening {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    reader = FrameReader(ser)
    print("Listening for target reports. Ctrl+C to stop.\n")

    try:
        while True:
            for payload in reader.read_frames():
                if args.debug_raw:
                    print(f"[{datetime.now().isoformat(timespec='milliseconds')}] "
                          f"RAW ({len(payload)} bytes): {payload.hex(' ')}")
                    continue

                targets = parse_targets(payload)
                if not targets:
                    continue

                now = time.time()
                for t in targets:
                    t.lane = lane_mapper.assign(t.lateral_m)
                    lane_label = t.lane if t.lane is not None else "?"

                    speed = t.speed_kmh
                    if args.use_fallback_speed and t.lane is not None:
                        est = speed_estimator.estimate(t.lane, t.distance_m, now)
                        if est is not None:
                            speed = est

                    ts = datetime.now().isoformat(timespec="milliseconds")
                    print(f"[{ts}] Lane {lane_label:>2}  "
                          f"dist={t.distance_m:6.2f} m  "
                          f"speed={speed:6.1f} km/h  "
                          f"angle={t.angle_deg:6.1f} deg")

                    if csv_writer:
                        csv_writer.writerow([ts, lane_label, f"{t.distance_m:.2f}",
                                              f"{speed:.1f}", f"{t.angle_deg:.1f}"])
                        csv_file.flush()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()