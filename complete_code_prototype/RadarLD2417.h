#ifndef RADAR_LD2417_H
#define RADAR_LD2417_H

#include <Arduino.h>

// ----------------------------------------------------------------------
// Structs & Global Variables
// ----------------------------------------------------------------------
struct Target {
  float distanceM;
  float speedKmh;
  float angleDeg;
  float lateralM;
  int8_t lane;
};

class RadarLD2417 {
public:
  // Constructor
  RadarLD2417();

  // Initialize the UART connection
  void begin(int rxPin, int txPin, long baudRate = 115200);

  // Read from serial and parse frames
  void poll();

  // Retrieve the latest valid target data
  Target getActiveTarget();

private:
  Target activeTarget;
  uint8_t compactFrame[16];
  uint8_t compactLen;

  // Radar Constants
  static const uint8_t FRAME_HEADER[2];
  static const uint8_t FRAME_TAIL[2];
  static const uint8_t MAX_TARGETS_PER_FRAME;
  static const uint8_t TARGET_RECORD_SIZE;
  static const uint8_t LANE_COUNT;
  static const float LANE_WIDTH_M;
  static const float CENTER_OFFSET_M;

  // Internal Parsing Logic
  int8_t assignLane(float lateralM);
  uint8_t parseCompactFrame(const uint8_t* frame, uint16_t len, Target* outTargets);
  void handleCompactFrame(const uint8_t* frame, uint16_t len);
};

#endif // RADAR_LD2417_H