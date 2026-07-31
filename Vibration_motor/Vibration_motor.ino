#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <NewPing.h>

// --- Pin Allocations ---
#define TRIGGER_PIN      5
#define ECHO_PIN         15
#define VIBRATOR_PIN     2

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

// --- Async Vibration State Machine ---
bool isVibrating = false;
int vibCyclesRemaining = 0;
unsigned long vibLastToggle = 0;
const unsigned long VIB_INTERVAL = 1000; 

// --- Shared Sensor Data ---
int globalSonar = 999;
int globalTof = 999;
bool tofReady = false;
bool mpuReady = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  
  if (lox.begin()) tofReady = true;

  GyroBus.begin(17, 16, 400000);
  GyroBus.beginTransmission(MPU_addr);
  GyroBus.write(0x6B); GyroBus.write(0);    
  if (GyroBus.endTransmission() == 0) mpuReady = true;

  pinMode(VIBRATOR_PIN, OUTPUT);
  Serial.println("--- System Initialized ---");
}

void processVibration() {
  if (!isVibrating) return;

  if (millis() - vibLastToggle >= VIB_INTERVAL) {
    vibLastToggle = millis();
    bool state = digitalRead(VIBRATOR_PIN);
    digitalWrite(VIBRATOR_PIN, !state);
    
    Serial.print("Vibration State: ");
    Serial.println(state ? "OFF" : "ON");
    
    vibCyclesRemaining--;
    if (vibCyclesRemaining <= 0) {
      digitalWrite(VIBRATOR_PIN, LOW);
      isVibrating = false;
      Serial.println("Vibration Sequence Complete.");
    }
  }
}

void startVibration(int cycles) {
  if (!isVibrating) {
    Serial.println("Collision Detected! Starting Vibration.");
    isVibrating = true;
    vibCyclesRemaining = cycles * 2;
    vibLastToggle = millis();
    digitalWrite(VIBRATOR_PIN, HIGH);
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
  globalSonar = (cm == 0) ? 999 : cm;
  if (tofReady) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    globalTof = (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 10 : 999;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    updateGyroCalculations((currentMillis - previousMillis) / 1000.0);
    previousMillis = currentMillis;
  }

  if (currentMillis - lastDistanceMillis >= distanceInterval) {
    updateDistances();
    lastDistanceMillis = currentMillis;
    
    // Telemetry
    Serial.print("P:"); Serial.print(pitch);
    Serial.print(" | R:"); Serial.print(roll);
    Serial.print(" | Sonar:"); Serial.print(globalSonar);
    Serial.print("cm | ToF:"); Serial.print(globalTof);
    Serial.println("cm");
  }

  processVibration();

  if (globalSonar <= 50 && globalTof <= 50) {
    startVibration(2);
  }
}