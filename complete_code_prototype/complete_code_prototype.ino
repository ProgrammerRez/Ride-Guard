#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "SafetyLogic.h"
#include "HapticController.h"
#include "RadarLD2417.h" // <-- Imported Radar Module

// ----------------------------------------------------------------------
// Hardware Configurations
// ----------------------------------------------------------------------
#define RADAR_RX_PIN        16   // ESP32 pin wired to LD2417 TX
#define RADAR_TX_PIN        17   // ESP32 pin wired to LD2417 RX
#define RADAR_BAUD          115200 // LD2417 factory default

#define MPU_ADDR            0x68
#define MPU_ALPHA           0.98f

// ----------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------
// IMU State
float pitchAngle = 0.0f;
float rollAngle = 0.0f;
bool mpuOnline = false;

// Driver Objects
SafetyLogic safetyEngine;
HapticController haptics(VIB_LEFT_PIN, VIB_RIGHT_PIN);
RadarLD2417 radar; // Instantiate the radar module

// Timing References
unsigned long prevMpuTime = 0;
const uint32_t MPU_INTERVAL_MS = 10; // 100Hz IMU Execution
unsigned long lastTelemetryPrint = 0;

// ----------------------------------------------------------------------
// IMU Functions
// ----------------------------------------------------------------------
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    mpuOnline = true;
    Serial.println(F("-> MPU6050 initialized successfully."));
  } else {
    Serial.println(F("-> MPU6050 initialization failed. Check I2C wiring (GPIO 21/22)."));
  }
}

void updateIMU(float dt) {
  if (!mpuOnline) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() < 14) return;

  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temp bytes
  int16_t GyX = Wire.read() << 8 | Wire.read();
  int16_t GyY = Wire.read() << 8 | Wire.read();

  float accPitch = atan2(AcY, AcZ) * RAD_TO_DEG;
  float accRoll  = atan2(AcX, sqrt((float)AcY * AcY + (float)AcZ * AcZ)) * RAD_TO_DEG;

  pitchAngle = MPU_ALPHA * (pitchAngle + (GyX / 131.0f) * dt) + (1.0f - MPU_ALPHA) * accPitch;
  rollAngle  = MPU_ALPHA * (rollAngle  + (GyY / 131.0f) * dt) + (1.0f - MPU_ALPHA) * accRoll;
}

// ----------------------------------------------------------------------
// Main Application Routines
// ----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\nHLK-LD2417 Integrated Rider Safety System Starting..."));

  // 1. Initialize Motors
  haptics.begin();

  // 2. Initialize IMU
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  initMPU();

  // 3. Initialize Radar Module
  radar.begin(RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD);
  Serial.println(F("-> Radar Hardware Serial Initialized @ 115200 Baud."));
}

void loop() {
  unsigned long now = millis();

  // Task 1: MPU6050 Gyro Filter Loop (100Hz)
  if (now - prevMpuTime >= MPU_INTERVAL_MS) {
    float dt = (now - prevMpuTime) / 1000.0f;
    prevMpuTime = now;
    updateIMU(dt);
  }

  // Task 2: Ingest UART Data using the Radar Module
  radar.poll();
  Target activeTarget = radar.getActiveTarget();

  // Task 3: Calculate Threat Assessment
  HazardAssessment hazard = safetyEngine.assessThreat(
    pitchAngle,
    rollAngle,
    activeTarget.distanceM,
    activeTarget.speedKmh,
    activeTarget.angleDeg
  );

  // Task 4: Drive Vibration Motor Alerts
  haptics.update(hazard);

  // Continuous Telemetry Output
  if (now - lastTelemetryPrint >= 200) {
    lastTelemetryPrint = now;
    Serial.printf("[%lu] Pitch: %5.1f° | Lean: %5.1f° | Dist: %6.2fm | Speed: %6.1f km/h | Hazard Level: %d\n",
                  now, pitchAngle, rollAngle, activeTarget.distanceM, activeTarget.speedKmh, (int)hazard.threatLevel);
  }
}