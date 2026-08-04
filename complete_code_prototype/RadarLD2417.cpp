#include "RadarLD2417.h"

#define RADAR_SERIAL Serial2

// Radar Constants Initialization
const uint8_t RadarLD2417::FRAME_HEADER[2] = {0xAA, 0xAA};
const uint8_t RadarLD2417::FRAME_TAIL[2]   = {0x55, 0x55};
const uint8_t RadarLD2417::MAX_TARGETS_PER_FRAME = 8;
const uint8_t RadarLD2417::TARGET_RECORD_SIZE = 7;
const uint8_t RadarLD2417::LANE_COUNT      = 3;
const float   RadarLD2417::LANE_WIDTH_M    = 3.5f;
const float   RadarLD2417::CENTER_OFFSET_M = -5.25f;

RadarLD2417::RadarLD2417() {
  compactLen = 0;
  activeTarget = {0.0f, 0.0f, 0.0f, 0.0f, -1};
}

void RadarLD2417::begin(int rxPin, int txPin, long baudRate) {
  RADAR_SERIAL.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

Target RadarLD2417::getActiveTarget() {
  return activeTarget;
}

int8_t RadarLD2417::assignLane(float lateralM) {
  float adjusted = lateralM - CENTER_OFFSET_M;
  int laneIndex = (int)floor(adjusted / LANE_WIDTH_M);
  if (laneIndex >= 0 && laneIndex < LANE_COUNT) {
    return laneIndex + 1;
  }
  return -1;
}

uint8_t RadarLD2417::parseCompactFrame(const uint8_t* frame, uint16_t len, Target* outTargets) {
  if (len < 10) return 0;

  uint8_t status = frame[2];
  if (status == 0x00) return 0;

  uint16_t rawDistance = frame[5] | (frame[6] << 8);
  uint16_t rawSpeed = frame[7] | (frame[8] << 8);

  if (rawDistance == 0 && rawSpeed == 0) return 0;

  Target t;
  t.distanceM = rawDistance / 100.0f;
  t.speedKmh  = rawSpeed / 100.0f;
  t.angleDeg  = 0.0f;
  t.lateralM  = 0.0f;
  t.lane      = -1;

  outTargets[0] = t;
  return 1;
}

void RadarLD2417::handleCompactFrame(const uint8_t* frame, uint16_t len) {
  if (len == 5 && frame[2] == 0x00) {
    activeTarget.distanceM = 0.0f;
    activeTarget.speedKmh = 0.0f;
    activeTarget.angleDeg = 0.0f;
    return;
  }

  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseCompactFrame(frame, len, targets);
  if (n == 0) return;

  activeTarget = targets[0];
}

void RadarLD2417::poll() {
  while (RADAR_SERIAL.available()) {
    uint8_t b = (uint8_t)RADAR_SERIAL.read();

    if (compactLen == 0 && b != FRAME_HEADER[0]) continue;
    if (compactLen == 1 && b != FRAME_HEADER[1]) { compactLen = 0; continue; }
    compactFrame[compactLen++] = b;
    if (compactLen >= sizeof(compactFrame)) { compactLen = 0; continue; }

    if (compactLen == 5 && compactFrame[2] == 0x00 &&
        compactFrame[3] == FRAME_TAIL[0] && compactFrame[4] == FRAME_TAIL[1]) {
      handleCompactFrame(compactFrame, 5);
      compactLen = 0;
    } else if (compactLen == 10) {
      handleCompactFrame(compactFrame, 10);
      compactLen = 0;
    }
  }
}