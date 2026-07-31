#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <NewPing.h>

// --- Pin Allocations ---
#define TRIGGER_PIN  5
#define ECHO_PIN     15   // Connect Echo here to avoid conflict with Servo 18
#define PITCH_SERVO_PIN 18
#define ROLL_SERVO_PIN  19
#define VIBRATOR_PIN 2

// Isolated Gyro Pins (Bus 1)
#define GYRO_SDA     17
#define GYRO_SCL     16

// --- Sensor Instantiations ---
NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
TwoWire GyroBus = TwoWire(1); // Second hardware I2C bus

// --- Gyro Variables ---
const int MPU_addr = 0x68;
const float alpha = 0.98;
float pitch = 0, roll = 0;

// --- Timing Variables ---
unsigned long previousMillis = 0;
const long interval = 10; // 100Hz Gyro Loop

unsigned long lastDistanceMillis = 0;
const long distanceInterval = 60; // 16Hz Sensor Loop (Safe for Sonar Echo)

// --- Shared Sensor Data ---
int globalSonar = 999;
int globalTof = 999;
bool tofReady = false;
bool mpuReady = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Initializing System ---");

  // 1. Setup default I2C bus for TOF (Pins 21/22)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (lox.begin()) {
    Serial.println("-> ToF Init: SUCCESS");
    tofReady = true;
  } else {
    Serial.println("-> ToF Init: FAILED");
  }

  // 2. Setup isolated hardware I2C bus for Gyro (Pins 17/16)
  GyroBus.begin(GYRO_SDA, GYRO_SCL, 400000);
  GyroBus.beginTransmission(MPU_addr);
  GyroBus.write(0x6B); 
  GyroBus.write(0);    
  if (GyroBus.endTransmission() == 0) {
    Serial.println("-> Gyro Init: SUCCESS");
    mpuReady = true;
  } else {
    Serial.println("-> Gyro Init: FAILED");
  }
  pinMode(VIBRATOR_PIN, OUTPUT);
  digitalWrite(VIBRATOR_PIN, LOW);
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
  GyroBus.read(); GyroBus.read(); // Skip Temp
  int16_t GyX = GyroBus.read()<<8|GyroBus.read();
  int16_t GyY = GyroBus.read()<<8|GyroBus.read();

  float accPitch = atan2(AcY, AcZ) * 180 / PI;
  float accRoll  = atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

  pitch = alpha * (pitch + (GyX / 131.0) * dt) + (1.0 - alpha) * accPitch;
  roll  = alpha * (roll  + (GyY / 131.0) * dt) + (1.0 - alpha) * accRoll;
}

void updateDistances() {
  // Read Sonar
  int cm = sonar.ping_cm();
  globalSonar = (cm == 0) ? 999 : cm;

  // Read ToF without blocking
  if (tofReady) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    globalTof = (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 10 : 999;
  }
}

void vibration(int cycles , int delayValue ){
  for (int i=0; i<=cycles; i++){
    digitalWrite(VIBRATOR_PIN, HIGH);
    delay(delayValue);
    digitalWrite(VIBRATOR_PIN, LOW);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Task 1: High-Speed Gyro Calculations (Every 10ms)
  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;
    updateGyroCalculations(dt);
  }

  // Task 2: Slower Distance Sampling (Every 60ms)
  // This allows Sonar sound waves to clear out, stopping false zeros
  if (currentMillis - lastDistanceMillis >= distanceInterval) {
    lastDistanceMillis = currentMillis;
    updateDistances();

    // Telemetry output
    Serial.print("Pitch:"); Serial.print(pitch);
    Serial.print(",Roll:"); Serial.print(roll);
    Serial.print(",Sonar:"); Serial.print(globalSonar);
    Serial.print(",ToF:"); Serial.println(globalTof);
  }

  if (globalSonar <= 50 && globalTof <= 50) {
    vibration(2, 1000);
  }
  
}