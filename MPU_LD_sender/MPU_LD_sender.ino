/*
  Device 1: HLK-LD2417 Radar + MPU6050 + ESP-NOW Sender
  =======================================================================
  Continuously streams radar status (danger state and closest distance) 
  to the receiver at 10Hz.
*/

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// ----------------------------------------------------------------------
// Code 1 (MPU6050) Variables
// ----------------------------------------------------------------------
const int MPU_addr = 0x68;
const float alpha = 0.98;
float pitch = 0, roll = 0;
unsigned long previousMillis = 0;
const long interval = 10; 

// ----------------------------------------------------------------------
// Code 2 (LD2417) CONFIG
// ----------------------------------------------------------------------
#define RADAR_SERIAL        Serial2
#define RADAR_RX_PIN        16   
#define RADAR_TX_PIN        17   
#define RADAR_BAUD          115200  

static const uint8_t FRAME_HEADER[2] = {0xAA, 0xAA};
static const uint8_t FRAME_TAIL[2]   = {0x55, 0x55};
static const uint8_t MAX_TARGETS_PER_FRAME = 8;
static const uint8_t TARGET_RECORD_SIZE = 7;  

static const uint8_t  LANE_COUNT     = 3;
static const float    LANE_WIDTH_M   = 3.5f;
static const float    CENTER_OFFSET_M = -5.25f;  

struct Target {
  float distanceM;
  float speedKmh;
  float angleDeg;
  float lateralM;
  int8_t lane;  
};

uint8_t compactFrame[16];
uint8_t compactLen = 0;

// ----------------------------------------------------------------------
// ESP-NOW / Streaming Configuration
// ----------------------------------------------------------------------
// REPLACE WITH THE MAC ADDRESS OF YOUR RECEIVER ESP32
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

typedef struct struct_message {
  bool vibrate;
  float closestDistance; // Added data stream
} struct_message;

struct_message alertData;
esp_now_peer_info_t peerInfo;

// Streaming variables
bool currentDanger = false;
float currentClosest = 0.0f;
unsigned long lastStreamTime = 0;
const unsigned long STREAM_INTERVAL = 100; // Stream data every 100ms (10Hz)

// ----------------------------------------------------------------------
// Helpers & Parsing
// ----------------------------------------------------------------------

int8_t assignLane(float lateralM) {
  float adjusted = lateralM - CENTER_OFFSET_M;
  int laneIndex = (int)floor(adjusted / LANE_WIDTH_M);  
  if (laneIndex >= 0 && laneIndex < LANE_COUNT) {
    return laneIndex + 1;  
  }
  return -1;
}

uint8_t parseCompactFrame(const uint8_t* frame, uint16_t len, Target* outTargets) {
  if (len < 10) return 0;
  if (frame[2] == 0x00) return 0;

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
    t.angleDeg  = (direction == 1) ? -45.0f : (direction == 2) ? 45.0f : 0.0f;
    t.lateralM  = t.distanceM * sinf(radians(t.angleDeg));
    t.lane      = assignLane(t.lateralM);

    outTargets[count++] = t;
    if (count >= MAX_TARGETS_PER_FRAME) break;
  }
  return count;
}

// Update the global state based on targets detected
void evaluateTargets(Target* targets, uint8_t n) {
  bool danger = false;
  float closest = 999.0f;
  
  for (uint8_t i = 0; i < n; i++) {
    if (targets[i].distanceM > 0.0f) {
      if (targets[i].distanceM < closest) {
        closest = targets[i].distanceM;
      }
      if (targets[i].distanceM < 10.0f) {
        danger = true;
      }
    }
  }
  
  currentDanger = danger;
  currentClosest = (closest == 999.0f) ? 0.0f : closest;
}

void handleCompactFrame(const uint8_t* frame, uint16_t len) {
  if (len == 5 && frame[2] == 0x00) {
    currentDanger = false;
    currentClosest = 0.0f;
    return;
  }

  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseCompactFrame(frame, len, targets);
  if (n > 0) evaluateTargets(targets, n);
}

void handleFrame(const uint8_t* payload, uint16_t len) {
  if (len >= 1 && payload[0] == 0) {
    currentDanger = false;
    currentClosest = 0.0f;
    return;
  }

  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseTargets(payload, len, targets);
  if (n > 0) evaluateTargets(targets, n);
}

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
// Setup & Loop
// ----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- ESP-NOW Setup ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Clear structure to avoid core v3 initialization issues
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW initialized.");

  // --- MPU6050 Setup ---
  Wire.begin(21, 22);
  Wire.setClock(400000); 
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
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}

void loop() {
  pollRadar();

  unsigned long currentMillis = millis();

  // Stream radar data continuously every 100ms
  if (currentMillis - lastStreamTime >= STREAM_INTERVAL) {
    lastStreamTime = currentMillis;
    alertData.vibrate = currentDanger;
    alertData.closestDistance = currentClosest;
    esp_now_send(receiverAddress, (uint8_t *) &alertData, sizeof(alertData));
  }

  // Handle MPU
  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);

    if (Wire.available() < 14) return;

    int16_t AcX = Wire.read()<<8|Wire.read();
    int16_t AcY = Wire.read()<<8|Wire.read();
    int16_t AcZ = Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read(); 
    int16_t GyX = Wire.read()<<8|Wire.read();
    int16_t GyY = Wire.read()<<8|Wire.read();

    float accPitch = atan2(AcY, AcZ) * 180 / PI;
    float accRoll  = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

    pitch = alpha * (pitch + (GyX / 131.0) * dt) + (1.0 - alpha) * accPitch;
    roll  = alpha * (roll  + (GyY / 131.0) * dt) + (1.0 - alpha) * accRoll;
  }
}
