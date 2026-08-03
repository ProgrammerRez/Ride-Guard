#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Hardware Pins ---
// MPU6050 IMU (Primary I2C Bus)
#define MPU_SDA_PIN          21
#define MPU_SCL_PIN          22

// LD2417 mmWave Radar (UART2)
#define RADAR_RX_PIN         16
#define RADAR_TX_PIN         17
#define RADAR_BAUD           256000

// Haptic Vibration Motors
#define VIB_LEFT_PIN         2
#define VIB_RIGHT_PIN        4

// --- System Thresholds & Calibration ---
#define LEAN_THRESHOLD_DEG   15.0f   // Minimum roll angle (degrees) to classify a turn
#define DANGER_DIST_CLOSE_M  2.5f    // Critical hazard proximity threshold (meters)
#define DANGER_DIST_FAR_M    6.0f    // Warning hazard proximity threshold (meters)
#define TTC_WARN_SEC         2.5f    // Time-To-Collision alert limit (seconds)

#endif
