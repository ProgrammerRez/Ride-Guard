#include <Arduino.h>

// ESP32 Hardware Serial 2 Pins
#define RX2_PIN 16 // Connect to HLK-LD2417 TX pin
#define TX2_PIN 17 // Connect to HLK-LD2417 RX pin

HardwareSerial RadarSerial(2);

uint8_t rxBuffer[64];
size_t bufIndex = 0;

void parseTargetFrame(uint8_t* frame, size_t len) {
  // Byte 2: Status Flag
  uint8_t status = frame[2];

  // 1. Handle Clear / No Target Frame (AA AA 00 55 55)
  if (status == 0x00) {
    Serial.println("[STATUS] Area Clear — No target detected.");
    return;
  }

  // 2. Handle Target Detected Frame
  if (len >= 10) {
    // Bytes 5-6: Distance in Centimeters (Little-Endian)
    uint16_t rawDistance = frame[5] | (frame[6] << 8);
    float distance_m = rawDistance / 100.0f;

    // Bytes 7-8: Speed / Intensity Value
    uint16_t rawSpeed = frame[7] | (frame[8] << 8);
    uint16_t speed_kmh = rawSpeed / 100; // Scaled speed estimate

    // Byte 9: Direction Flag (0x01 = Approaching, 0x02 / 0x03 = Receding)
    uint8_t dirByte = frame[9];
    String directionStr = "Stationary / Unknown";
    if (dirByte == 0x01) {
      directionStr = "Approaching (Towards Radar)";
    } else if (dirByte == 0x02 || dirByte == 0x03) {
      directionStr = "Receding (Away from Radar)";
    }

    // Output Telemetry
    Serial.println("================ TARGET DETECTED ================");
    Serial.print(" Distance : "); Serial.print(distance_m, 2); Serial.println(" meters");
    Serial.print(" Speed    : "); Serial.print(speed_kmh); Serial.println(" km/h");
    Serial.print(" Direction: "); Serial.println(directionStr);
    Serial.println("=================================================\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n-------------------------------------------------");
  Serial.println("  HLK-LD2417 Compact Frame Parser Active (ESP32) ");
  Serial.println("-------------------------------------------------");

  // Initialize UART2
  RadarSerial.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
}

void loop() {
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();

    // Align to Header (0xAA 0xAA)
    if (bufIndex == 0 && b != 0xAA) continue;
    if (bufIndex == 1 && b != 0xAA) { bufIndex = 0; continue; }

    rxBuffer[bufIndex++] = b;

    // Prevent Buffer Overflow
    if (bufIndex >= sizeof(rxBuffer)) {
      bufIndex = 0;
      continue;
    }

    // Check for 5-byte Idle Frame (AA AA 00 55 55)
    if (bufIndex == 5 && rxBuffer[2] == 0x00 && rxBuffer[3] == 0x55 && rxBuffer[4] == 0x55) {
      parseTargetFrame(rxBuffer, 5);
      bufIndex = 0;
    } 
    // Check for 10-byte Active Target Frame
    else if (bufIndex == 10) {
      parseTargetFrame(rxBuffer, 10);
      bufIndex = 0;
    }
  }
}