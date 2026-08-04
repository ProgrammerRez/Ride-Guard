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

#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// ----------------------------------------------------------------------
// Code 1 (MPU6050) Variables
// ----------------------------------------------------------------------
const int MPU_addr = 0x68;

// Fixed weights: 98% Gyro (kills fast shakes), 2% Accel (keeps it level long-term)
const float alpha = 0.98;
float pitch = 0, roll = 0;

unsigned long previousMillis = 0;
const long interval = 10; // 100Hz execution rate


// ----------------------------------------------------------------------
// Code 2 (LD2417) CONFIG - verify these against your datasheet (see header comment)
// ----------------------------------------------------------------------
#define RADAR_SERIAL        Serial2
#define RADAR_RX_PIN        16   // ESP32 pin wired to LD2417 TX
#define RADAR_TX_PIN        17   // ESP32 pin wired to LD2417 RX
#define RADAR_BAUD          115200  // LD2417 factory default (not LD2451's 256000)

static const uint8_t FRAME_HEADER[2] = {0xAA, 0xAA};
static const uint8_t FRAME_TAIL[2]   = {0x55, 0x55};

static const uint8_t MAX_TARGETS_PER_FRAME = 8;
static const uint8_t TARGET_RECORD_SIZE = 7;  // dir(1) + dist(2) + speed(2) + status(2)

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

// Compact frame reassembly buffer
uint8_t compactFrame[16];
uint8_t compactLen = 0;

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

// Parses LD2417 compact 10-byte target frame (AA AA ... no trailing 55 55).
// Bytes 5-6: distance (m * 100), 7-8: speed (km/h * 100), 9: direction flag.
uint8_t parseCompactFrame(const uint8_t* frame, uint16_t len, Target* outTargets) {
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
  t.lane      = -1;  // compact frame has no lateral angle — tune lane mapping separately

  outTargets[0] = t;
  return 1;
}

// Parses LD2417 multi-target payload: [target_count][7-byte records...] 55 55 delimited.
uint8_t parseTargets(const uint8_t* payload, uint16_t payloadLen, Target* outTargets) {
  if (payloadLen < 1) return 0;

  uint8_t targetCount = payload[0];
  if (targetCount == 0) return 0;
  if (targetCount > MAX_TARGETS_PER_FRAME) targetCount = MAX_TARGETS_PER_FRAME;

  uint16_t expectedLen = 1 + (uint16_t)targetCount * TARGET_RECORD_SIZE;
  if (payloadLen < expectedLen) targetCount = (payloadLen - 1) / TARGET_RECORD_SIZE;

  uint8_t count = 0;
  uint16_t offset = 1;
  for (uint8_t i = 0; i < targetCount; i++) {
    uint8_t direction = payload[offset];
    uint16_t rawDistance = payload[offset + 1] | (payload[offset + 2] << 8);
    uint16_t rawSpeed = payload[offset + 3] | (payload[offset + 4] << 8);
    offset += TARGET_RECORD_SIZE;

    if (rawDistance == 0 && rawSpeed == 0) continue;

    Target t;
    t.distanceM = rawDistance / 100.0f;
    t.speedKmh  = rawSpeed / 100.0f;
    // LD2417 reports left/right bearing, not degrees — use nominal angle for lane math.
    t.angleDeg  = (direction == 1) ? -45.0f : (direction == 2) ? 45.0f : 0.0f;
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

void handleCompactFrame(const uint8_t* frame, uint16_t len) {
  if (DEBUG_RAW) {
    printFrameHex(frame, len);
    return;
  }

  if (len == 5 && frame[2] == 0x00) {
    Serial.printf("[%lu] Area clear — no target detected.\n", millis());
    return;
  }

  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseCompactFrame(frame, len, targets);
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
  }
}

void handleFrame(const uint8_t* payload, uint16_t len) {
  if (DEBUG_RAW) {
    printFrameHex(payload, len);
    return;
  }

  if (len >= 1 && payload[0] == 0) {
    Serial.printf("[%lu] Area clear — no target detected.\n", millis());
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
// Serial ingestion: sync on AA AA frames
// ----------------------------------------------------------------------

void pollRadar() {
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

// ----------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------

void setup() {
  // --- MPU6050 Setup ---
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Booting Standard Gyro Sensor ---");

  Wire.begin(21, 22);
  Wire.setClock(400000); // Fast I2C clock

  // Wake up MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0);    
  if (Wire.endTransmission() != 0) {
    while(1) {
      Serial.println("CRITICAL: Sensor not found on Pins 21/22!");
      delay(1000);
    }
  }

  previousMillis = millis();

  // --- LD2417 Setup ---
  delay(200);
  Serial.println("HLK-LD2417 traffic monitor starting...");

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  if (DEBUG_RAW) {
    Serial.println("DEBUG_RAW is ON -- printing raw frame hex. "
                   "Compare against your datasheet, then set DEBUG_RAW=false.");
  }
}

void loop() {
  // --- LD2417 Radar Polling ---
  pollRadar();
  // No delay needed -- pollRadar() only blocks on available serial data.

  // --- MPU6050 Gyro Processing ---
  unsigned long currentMillis = millis();

  // Non-blocking timer
  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    // Read 6-axis raw data
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);

    if (Wire.available() < 14) return;

    int16_t AcX = Wire.read()<<8|Wire.read();
    int16_t AcY = Wire.read()<<8|Wire.read();
    int16_t AcZ = Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read(); // Skip Temp
    int16_t GyX = Wire.read()<<8|Wire.read();
    int16_t GyY = Wire.read()<<8|Wire.read();

    // 1. Calculate stable baseline angles from Accelerometer
    float accPitch = atan2(AcY, AcZ) * 180 / PI;
    float accRoll  = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

    // 2. Standard Complementary Filter Integration
    pitch = alpha * (pitch + (GyX / 131.0) * dt) + (1.0 - alpha) * accPitch;
    roll  = alpha * (roll  + (GyY / 131.0) * dt) + (1.0 - alpha) * accRoll;

    // Telemetry output for Arduino Serial Plotter
    Serial.print("Pitch_Angle:"); Serial.print(pitch);
    Serial.print(",Roll_Angle:");  Serial.println(roll);
  }
}
