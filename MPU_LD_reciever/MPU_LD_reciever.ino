/*
  Device 2: ESP-NOW Receiver + Extended Vibration
  =======================================================================
  Receives streaming data. If a target triggers a vibration alert, the 
  motor stays ON for a guaranteed minimum time (e.g., 2 seconds).
*/

#include <esp_now.h>
#include <WiFi.h>

#define MOTOR_PIN 27 

// Must match sender exactly
typedef struct struct_message {
    bool vibrate;
    float closestDistance;
} struct_message;

struct_message myData;
unsigned long lastRecvTime = 0;

// Variables for extending the vibration time
unsigned long motorTurnOffTime = 0;
const unsigned long VIBRATION_DURATION_MS = 2000; // 2 Seconds minimum run time

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();
  
  if (myData.vibrate) {
    // Keep pushing the turn-off time further into the future
    motorTurnOffTime = millis() + VIBRATION_DURATION_MS;
    Serial.printf("Motor ON  | Target at: %.2f m\n", myData.closestDistance);
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

  // Failsafe: if connection to sender is completely lost for 2 seconds
  if (now - lastRecvTime > 2000) {
    digitalWrite(MOTOR_PIN, LOW);
  } else {
    // If we are currently within the vibration window, turn it on
    if (now < motorTurnOffTime) {
      digitalWrite(MOTOR_PIN, HIGH);
    } else {
      digitalWrite(MOTOR_PIN, LOW);
    }
  }
  
  delay(10); // Small delay to prevent watchdog reset
}
