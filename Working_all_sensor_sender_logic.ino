#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <NewPing.h>
#include <esp_now.h>
#include <WiFi.h>

// --- ESP-NOW Broadcast Address ---
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- Telemetry Structure ---
typedef struct struct_message {
    float pitch;
    float roll;
    int sonar;
    int tof;
} struct_message;

struct_message sensorData;

// --- Pin Allocations ---
#define TRIGGER_PIN  5
#define ECHO_PIN     15   
#define PITCH_SERVO_PIN 18
#define ROLL_SERVO_PIN  19
#define VIBRATOR_PIN 2

// Isolated Gyro Pins (Bus 1)
#define GYRO_SDA     17
#define GYRO_SCL     16

// --- Sensor Instantiations ---
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
TwoWire GyroBus = TwoWire(1); 

// --- Gyro Variables ---
const int MPU_addr = 0x68;
const float alpha = 0.98;
float pitch = 0, roll = 0;

// --- Timing Variables ---
unsigned long previousMillis = 0;
const long interval = 10; 

unsigned long lastDistanceMillis = 0;
const long distanceInterval = 60; 

bool tofReady = false;
bool mpuReady = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  // Register Peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // 1. Setup default I2C bus for TOF (Pins 21/22)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (lox.begin()) {
    tofReady = true;
  }

  // 2. Setup isolated hardware I2C bus for Gyro (Pins 17/16)
  GyroBus.begin(GYRO_SDA, GYRO_SCL, 400000);
  GyroBus.beginTransmission(MPU_addr);
  GyroBus.write(0x6B); 
  GyroBus.write(0);    
  if (GyroBus.endTransmission() == 0) {
    mpuReady = true;
  }
}

void updateGyroCalculations(float dt) {
  if (!mpuReady) return;

  GyroBus.beginTransmission(MPU_addr);
  GyroBus.write(0x3B);
  GyroBus.endTransmission(false);
  GyroBus.requestFrom(MPU_addr, 14, true);

  if (GyroBus.available() < 14) return;

  int16_t AcX = GyroBus.read()<<8|GyroBus.read();
  int16_t AcY = GyroBus.read()<<8|GyroBus.read();
  int16_t AcZ = GyroBus.read()<<8|GyroBus.read();
  GyroBus.read(); GyroBus.read(); 
  int16_t GyX = GyroBus.read()<<8|GyroBus.read();
  int16_t GyY = GyroBus.read()<<8|GyroBus.read();

  float accPitch = atan2(AcY, AcZ) * 180 / PI;
  float accRoll  = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

  pitch = alpha * (pitch + (GyX / 131.0) * dt) + (1.0 - alpha) * accPitch;
  roll  = alpha * (roll  + (GyY / 131.0) * dt) + (1.0 - alpha) * accRoll;
}

void updateDistances() {
  int cm = sonar.ping_cm();
  sensorData.sonar = (cm == 0) ? 999 : cm;

  if (tofReady) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    sensorData.tof = (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 10 : 999;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;
    updateGyroCalculations(dt);
  }

  if (currentMillis - lastDistanceMillis >= distanceInterval) {
    lastDistanceMillis = currentMillis;
    updateDistances();

    // Fill struct for transmission
    sensorData.pitch = pitch;
    sensorData.roll = roll;

    // Broadcast current sensor state to receivers
    esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));

    Serial.print("P:"); Serial.print(sensorData.pitch);
    Serial.print(" S:"); Serial.print(sensorData.sonar);
    Serial.print(" T:"); Serial.println(sensorData.tof);
  }
}