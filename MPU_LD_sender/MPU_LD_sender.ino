/*
  Device 1: HLK-LD2417 Radar + MPU6050 + ESP-NOW Sender
  Modularized Architecture: Calculate -> Decide -> Output
*/

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// --- HARDWARE & TIMING CONSTANTS ---
const int MPU_addr = 0x68;
const float alpha = 0.98;
const float TURN_THRESHOLD = 35.0; 
const long IMU_INTERVAL = 10; 
const unsigned long STREAM_INTERVAL = 100; 

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

// REPLACE WITH THE MAC ADDRESS OF YOUR RECEIVER ESP32
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// --- DATA STRUCTURES ---
struct Target {
  float distanceM;
  float speedKmh;
  float angleDeg;
  float lateralM;
  int8_t lane;  
};

// Internal struct to hold all live telemetry
struct SensorData {
  float pitch;
  float roll;
  float turnRate;
  bool isTurning;
  float closestDistance;
  uint8_t targetCount;
  Target targets[MAX_TARGETS_PER_FRAME];
};

// ESP-NOW Payload struct
typedef struct struct_message {
  bool vibrate;
  float closestDistance;
  float pitch;
  float roll;
  float turnRate;
  bool isTurning;
  uint8_t targetCount;
  Target targets[MAX_TARGETS_PER_FRAME];
} struct_message;

// Decision Literal State
enum DecisionState {
  STATE_SAFE,
  STATE_DANGER
};

// --- GLOBALS ---
uint8_t compactFrame[16];
uint8_t compactLen = 0;
struct_message alertData;
esp_now_peer_info_t peerInfo;


// ==========================================
// RADAR PARSING UTILITIES
// ==========================================

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

// Maps parsed radar frames into the live SensorData object
void processFrameData(Target* targets, uint8_t n, SensorData* data) {
  float closest = 999.0f;
  data->targetCount = n;
  
  for (uint8_t i = 0; i < n; i++) {
    data->targets[i] = targets[i];
    if (targets[i].distanceM > 0.0f && targets[i].distanceM < closest) {
      closest = targets[i].distanceM;
    }
  }
  data->closestDistance = (closest == 999.0f) ? 0.0f : closest;
}

void handleCompactFrame(const uint8_t* frame, uint16_t len, SensorData* data) {
  if (len == 5 && frame[2] == 0x00) {
    data->closestDistance = 0.0f;
    data->targetCount = 0;
    return;
  }
  Target targets[MAX_TARGETS_PER_FRAME];
  uint8_t n = parseCompactFrame(frame, len, targets);
  if (n > 0) processFrameData(targets, n, data);
}

void pollRadar(SensorData* data) {
  while (RADAR_SERIAL.available()) {
    uint8_t b = (uint8_t)RADAR_SERIAL.read();

    if (compactLen == 0 && b != FRAME_HEADER[0]) continue;
    if (compactLen == 1 && b != FRAME_HEADER[1]) { compactLen = 0; continue; }
    compactFrame[compactLen++] = b;
    if (compactLen >= sizeof(compactFrame)) { compactLen = 0; continue; }

    if (compactLen == 5 && compactFrame[2] == 0x00 &&
        compactFrame[3] == FRAME_TAIL[0] && compactFrame[4] == FRAME_TAIL[1]) {
      handleCompactFrame(compactFrame, 5, data);
      compactLen = 0;
    } else if (compactLen == 10) {
      handleCompactFrame(compactFrame, 10, data);
      compactLen = 0;
    }
  }
}


// ==========================================
// 1. CALCULATION FUNCTION
// ==========================================
SensorData calculateSensorData() {
  static SensorData data = {0}; // Persists state across loops
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  // Update Radar Data (Modifies the 'data' struct directly via reference)
  pollRadar(&data);

  // Update IMU Data based on interval
  if (currentMillis - previousMillis >= IMU_INTERVAL) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);

    if (Wire.available() >= 14) {
      int16_t AcX = Wire.read()<<8|Wire.read();
      int16_t AcY = Wire.read()<<8|Wire.read();
      int16_t AcZ = Wire.read()<<8|Wire.read();
      Wire.read(); Wire.read(); // Skip temp
      int16_t GyX = Wire.read()<<8|Wire.read();
      int16_t GyY = Wire.read()<<8|Wire.read();
      int16_t GyZ = Wire.read()<<8|Wire.read();

      data.turnRate = GyZ / 131.0; 
      data.isTurning = (abs(data.turnRate) > TURN_THRESHOLD);

      float accPitch = atan2(AcY, AcZ) * 180 / PI;
      float accRoll  = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

      data.pitch = alpha * (data.pitch + (GyX / 131.0) * dt) + (1.0 - alpha) * accPitch;
      data.roll  = alpha * (data.roll  + (GyY / 131.0) * dt) + (1.0 - alpha) * accRoll;
    }
  }
  
  return data; // Returns the unified sensor packet
}


// ==========================================
// 2. DECISION FUNCTION
// ==========================================
DecisionState makeDecision(SensorData data) {
  // Evaluates telemetry and passes back a state literal
  if (data.closestDistance > 0.0f && data.closestDistance < 10.0f) {
    return STATE_DANGER; 
  }
  
  return STATE_SAFE;
}


// ==========================================
// 3. OUTPUT FUNCTION
// ==========================================
void transmitData(SensorData data, DecisionState state) {
  static unsigned long lastStreamTime = 0;
  unsigned long currentMillis = millis();

  // Only stream at defined intervals to prevent ESP-NOW flooding
  if (currentMillis - lastStreamTime >= STREAM_INTERVAL) {
    lastStreamTime = currentMillis;

    // Apply the Decision State
    alertData.vibrate = (state == STATE_DANGER);
    
    // Apply the Telemetry Data
    alertData.closestDistance = data.closestDistance;
    alertData.pitch = data.pitch;
    alertData.roll = data.roll;
    alertData.turnRate = data.turnRate;
    alertData.isTurning = data.isTurning;
    alertData.targetCount = data.targetCount;
    
    for (uint8_t i = 0; i < data.targetCount; i++) {
      alertData.targets[i] = data.targets[i];
    }

    // Transmit
    esp_now_send(receiverAddress, (uint8_t *) &alertData, sizeof(alertData));
  }
}


// ==========================================
// SYSTEM SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  Wire.begin(21, 22);
  Wire.setClock(400000); 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission();

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}

void loop() {
  // 1. Calculation
  SensorData currentData = calculateSensorData();

  // 2. Decision 
  DecisionState currentState = makeDecision(currentData);

  // 3. Output
  transmitData(currentData, currentState);
}
