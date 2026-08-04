/*
  Device 2: ESP-NOW Receiver + Extended Vibration + Full Telemetry Processing
  Receives radar targets, MPU orientation angles, gyro rates, and turn status.
*/

#include <esp_now.h>
#include <WiFi.h>

#define MOTOR_PIN 27 

struct Target {
  float distanceM;
  float speedKmh;
  float angleDeg;
  float lateralM;
  int8_t lane;  
};

// Must exactly match the Sender's updated struct layout
typedef struct struct_message {
    bool vibrate;
    float closestDistance;
    float pitch;
    float roll;
    float gyroZ;
    bool isTurning;
    uint8_t targetCount;
    Target targets[8];
} struct_message;

struct_message myData;
unsigned long lastRecvTime = 0;

unsigned long motorTurnOffTime = 0;
const unsigned long VIBRATION_DURATION_MS = 2000; // Minimum 2 second pulse

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();
  
  // Print status of the vehicle and motion metrics
  Serial.printf("[Telemetry] Pitch: %.1f° | Roll: %.1f° | GyroZ: %.1f°/s | Turning: %s\n", 
                myData.pitch, myData.roll, myData.gyroZ, myData.isTurning ? "YES" : "NO");

  if (myData.vibrate) {
    motorTurnOffTime = millis() + VIBRATION_DURATION_MS;
    Serial.printf(">>> ALERT! Closest Target: %.2f m (Total targets: %u)\n", myData.closestDistance, myData.targetCount);
    
    for(uint8_t i = 0; i < myData.targetCount; i++) {
      Serial.printf("    -> Target %d: Dist=%.2fm, Speed=%.1fkm/h, Lane=%d\n", 
        i, myData.targets[i].distanceM, myData.targets[i].speedKmh, myData.targets[i].lane);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  WiFi.mode(WIFI_STA);
  
  Serial.println();
  Serial.print(">>> RECEIVER MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  unsigned long now = millis();

  // Failsafe: Turn off motor if connection is lost for more than 2 seconds
  if (now - lastRecvTime > 2000) {
    digitalWrite(MOTOR_PIN, LOW);
  } else {
    if (now < motorTurnOffTime) {
      digitalWrite(MOTOR_PIN, HIGH);
    } else {
      digitalWrite(MOTOR_PIN, LOW);
    }
  }
  
  delay(10);
}
