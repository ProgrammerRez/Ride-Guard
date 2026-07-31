/*
  HLK-LD2417 Vehicle Distance / Lane / Speed Monitor -- ESP32 / Arduino
  =======================================================================

  Reads target reports from a Hi-Link HLK-LD2417 (24GHz vehicle-status
  radar) over UART2 and reports, for every detected vehicle:
      - distance (m)
      - speed (km/h)
      - lane number (derived from lateral position)

  IMPORTANT - READ BEFORE USING
  ------------------------------
  Hi-Link revises the exact byte layout of the report frame across
  firmware/module revisions, and I could not pull a byte-verified copyj
  of the LD2417's datasheet. The FRAME_HEADER/FRAME_TAIL bytes and the
  field offsets in parseTargets() are best-effort based on the public
  Hi-Link vehicle-radar family (LD2451) and the general Hi-Link "LD"
  report structure. **Before trusting the output, confirm against the
  datasheet PDF that shipped with your module:**

      1. FRAME_HEADER / FRAME_TAIL byte sequences
      2. Little-endian vs big-endian fields
      3. Byte offset/width of: target count, distance, speed, angle,
         direction flag, and (if present) a raw lane index
      4. Whether angle is signed degrees, or already given some other way

  Set DEBUG_RAW to true first. It prints the raw hex of every frame the
  module sends over the USB serial monitor -- compare that against your
  datasheet's frame diagram and adjust the constants/parseTargets()
  below to match.

  Wiring (ESP32 UART2, adjust pins as needed)
  --------------------------------------------
      LD2417 TX  -> ESP32 GPIO16 (RX2)
      LD2417 RX  -> ESP32 GPIO17 (TX2)
      LD2417 VCC -> 5V  (check your module's voltage requirement)
      LD2417 GND -> GND
      NOTE: LD2417 UART logic level is typically 3.3V -- verify before
      wiring directly; the ESP32's UART pins are 3.3V logic.

  Dependencies
  ------------
  None beyond the ESP32 Arduino core (HardwareSerial is built in).

  Optional: install the ArduinoJson or SD library yourself if you want
  to log to an SD card or emit JSON -- left out here to keep this
  dependency-free; see the TODO near loop() for where to hook that in.
*/

#include <Arduino.h>
#include <math.h>

// ----------------------------------------------------------------------
// CONFIG - verify these against your datasheet (see header comment)
// ----------------------------------------------------------------------

#define RADAR_SERIAL        Serial2
#define RADAR_RX_PIN        16   // ESP32 pin wired to LD2417 TX
#define RADAR_TX_PIN        17   // ESP32 pin wired to LD2417 RX
#define RADAR_BAUD          256000  // Hi-Link "LD" series default

static const uint8_t FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};  // <-- VERIFY
static const uint8_t FRAME_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};  // <-- VERIFY

static const uint8_t MAX_TARGETS_PER_FRAME = 8;
static const uint16_t RX_BUF_SIZE = 512;   // ring buffer for incoming bytes

// Byte layout of a single target record -- VERIFY against datasheet.
// Assumed: 3x signed little-endian int16: distance_cm, speed_kmh, angle_deg
struct TargetRecordRaw {
  int16_t distance_cm;
  int16_t speed_kmh;
  int16_t angle_deg;
};
static const uint8_t TARGET_RECORD_SIZE = sizeof(TargetRecordRaw);  // 6 bytes

// Lane mapping config -- tune to your physical mounting
static const uint8_t  LANE_COUNT     = 3;
static const float    LANE_WIDTH_M   = 3.5f;
static const float    CENTER_OFFSET_M = -5.25f;  // lateral dist. from boresight to start of lane 1

// Set true to print raw frame hex instead of parsed targets (do this first!)
static const bool DEBUG_RAW = false;

// Set true to estimate speed from distance-over-time per lane instead of
// trusting the module's speed field (useful if firmware reports 0 unless
// "engineering mode" is enabled)
static const bool USE_FALLBACK_SPEED = false;

// ----------------------------------------------------------------------

struct Target {
  float distanceM;
  float speedKmh;
  float angleDeg;
  float lateralM;
  int8_t lane;  // -1 = outside configured lane span
};

// Simple ring buffer for accumulating serial bytes until we find a full frame
uint8_t rxBuf[RX_BUF_SIZE];
uint16_t rxLen = 0;

// Per-lane distance history for fallback speed estimation
static const uint8_t HISTORY_LEN = 5;
struct LaneHistory {
  unsigned long t[HISTORY_LEN];
  float d[HISTORY_LEN];
  uint8_t count = 0;
  uint8_t head = 0;
};
LaneHistory laneHistory[LANE_COUNT + 1];  // 1-indexed, index 0 unused

// ----------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------

int findSequence(const uint8_t* buf, uint16_t bufLen, const uint8_t* seq, uint8_t seqLen, int startAt = 0) {
  if (bufLen < seqLen) return -1;
  for (int i = startAt; i <= (int)bufLen - (int)seqLen; i++) {
    bool match = true;
    for (uint8_t j = 0; j < seqLen; j++) {
      if (buf[i + j] != seq[j]) { match = false; break; }
    }
    if (match) return i;
  }
  return -1;
}

int8_t assignLane(float lateralM) {
  float adjusted = lateralM - CENTER_OFFSET_M;
  int laneIndex = (int)floor(adjusted / LANE_WIDTH_M);  // 0-indexed
  if (laneIndex >= 0 && laneIndex < LANE_COUNT) {
    return laneIndex + 1;  // 1-indexed for humans
  }
  return -1;
}

float estimateFallbackSpeedKmh(int8_t lane, float distanceM, unsigned long nowMs) {
  if (lane < 1 || lane > LANE_COUNT) return NAN;
  LaneHistory& h = laneHistory[lane];

  h.t[h.head] = nowMs;
  h.d[h.head] = distanceM;
  h.head = (h.head + 1) % HISTORY_LEN;
  if (h.count < HISTORY_LEN) h.count++;

  if (h.count < 2) return NAN;

  // oldest entry is at h.head when buffer is full; if not full, oldest is index 0
  uint8_t oldestIdx = (h.count < HISTORY_LEN) ? 0 : h.head;
  uint8_t newestIdx = (h.head + HISTORY_LEN - 1) % HISTORY_LEN;

  unsigned long dtMs = h.t[newestIdx] - h.t[oldestIdx];
  if (dtMs == 0) return NAN;

  float ddM = h.d[oldestIdx] - h.d[newestIdx];  // positive = approaching
  float speedMps = ddM / (dtMs / 1000.0f);
  return speedMps * 3.6f;
}

// ----------------------------------------------------------------------
// Frame parsing
// ----------------------------------------------------------------------

// Parses one frame payload (bytes between header and tail) into targets.
// Returns the number of targets written into outTargets (capped at MAX_TARGETS_PER_FRAME).
uint8_t parseTargets(const uint8_t* payload, uint16_t payloadLen, Target* outTargets) {
  uint8_t count = 0;
  uint16_t nRecords = payloadLen / TARGET_RECORD_SIZE;
  if (nRecords > MAX_TARGETS_PER_FRAME) nRecords = MAX_TARGETS_PER_FRAME;

  for (uint16_t i = 0; i < nRecords; i++) {
    TargetRecordRaw raw;
    memcpy(&raw, payload + i * TARGET_RECORD_SIZE, sizeof(raw));

    // All-zero record usually means "empty slot" -- skip it.
    if (raw.distance_cm == 0 && raw.speed_kmh == 0 && raw.angle_deg == 0) continue;

    Target t;
    t.distanceM = raw.distance_cm / 100.0f;   // assumed cm -> m, VERIFY
    t.speedKmh  = (float)raw.speed_kmh;        // assumed already km/h, VERIFY
    t.angleDeg  = (float)raw.angle_deg;
    t.lateralM  = t.distanceM * sinf(radians(t.angleDeg));
    t.lane      = assignLane(t.lateralM);

    outTargets[count++] = t;
    if (count >= MAX_TARGETS_PER_FRAME) break;
  }
  return count;
}

void printFrameHex(const uint8_t* payload, uint16_t len) {
  Serial.printf("[%lu] RAW (%u bytes): ", millis(), len);
  for (uint16_t i = 0; i < len; i++) {
    Serial.printf("%02X ", payload[i]);
  }
  Serial.println();
}

void handleFrame(const uint8_t* payload, uint16_t len) {
  if (DEBUG_RAW) {
    printFrameHex(payload, len);
    return;
  }

  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseTargets(payload, len, targets);
  if (n == 0) return;

  unsigned long now = millis();
  for (uint8_t i = 0; i < n; i++) {
    Target& t = targets[i];

    float speed = t.speedKmh;
    if (USE_FALLBACK_SPEED && t.lane > 0) {
      float est = estimateFallbackSpeedKmh(t.lane, t.distanceM, now);
      if (!isnan(est)) speed = est;
    }

    Serial.printf("[%lu] Lane %s  dist=%6.2f m  speed=%6.1f km/h  angle=%6.1f deg\n",
                  now,
                  (t.lane > 0) ? String(t.lane).c_str() : "?",
                  t.distanceM, speed, t.angleDeg);

    // TODO: hook in SD-card logging, MQTT publish, HTTP POST, etc. here.
  }
}

// ----------------------------------------------------------------------
// Serial ingestion: pull bytes into rxBuf, scan for complete frames
// ----------------------------------------------------------------------

void pollRadar() {
  while (RADAR_SERIAL.available()) {
    if (rxLen >= RX_BUF_SIZE) {
      // Buffer full without finding a valid frame -- reset to avoid lockup.
      rxLen = 0;
    }
    rxBuf[rxLen++] = (uint8_t)RADAR_SERIAL.read();
  }

  while (true) {
    int start = findSequence(rxBuf, rxLen, FRAME_HEADER, sizeof(FRAME_HEADER));
    if (start == -1) {
      // No header found; if buffer is getting large, clear stale data.
      if (rxLen > RX_BUF_SIZE - 32) rxLen = 0;
      return;
    }
    int end = findSequence(rxBuf, rxLen, FRAME_TAIL, sizeof(FRAME_TAIL), start + sizeof(FRAME_HEADER));
    if (end == -1) {
      // Header found but tail hasn't arrived yet. Discard junk before
      // the header and wait for more bytes.
      if (start > 0) {
        memmove(rxBuf, rxBuf + start, rxLen - start);
        rxLen -= start;
      }
      return;
    }

    uint16_t payloadStart = start + sizeof(FRAME_HEADER);
    uint16_t payloadLen = end - payloadStart;
    handleFrame(rxBuf + payloadStart, payloadLen);

    uint16_t consumed = end + sizeof(FRAME_TAIL);
    memmove(rxBuf, rxBuf + consumed, rxLen - consumed);
    rxLen -= consumed;
  }
}

// ----------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("HLK-LD2417 traffic monitor starting...");

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  if (DEBUG_RAW) {
    Serial.println("DEBUG_RAW is ON -- printing raw frame hex. "
                    "Compare against your datasheet, then set DEBUG_RAW=false.");
  }
}

void loop() {
  pollRadar();
  // No delay needed -- pollRadar() only blocks on available serial data.
}
