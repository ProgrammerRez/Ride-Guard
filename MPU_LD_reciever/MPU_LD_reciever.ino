/*
  Device 2: ESP-NOW Receiver + Haptic Language System
  Receives radar targets and IMU data to drive dynamic vibration patterns based on threat severity.
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

// Must exactly match the Sender's struct layout
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

// Timing and State Variables
unsigned long lastRecvTime = 0;
unsigned long motorTurnOffTime = 0;
const unsigned long VIBRATION_LIFESPAN_MS = 1500; 

// Haptic Pattern Variables
uint8_t threatLevel = 0; // 0=Off, 1=Caution, 2=Warning, 3=Critical
unsigned long hapticPreviousMillis = 0;
bool hapticState = false;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();
  
  Serial.printf("[Telemetry] Pitch: %.1f° | Roll: %.1f° | GyroZ: %.1f°/s | Turning: %s\n", 
                myData.pitch, myData.roll, myData.gyroZ, myData.isTurning ? "YES" : "NO");

  if (myData.vibrate && myData.closestDistance > 0.0f) {
    motorTurnOffTime = millis() + VIBRATION_LIFESPAN_MS;
    
    // Evaluate leaning/turning vulnerability
    bool isLeaning = (abs(myData.roll) > 20.0f) || (abs(myData.pitch) > 40.0f);
    bool corneringDanger = myData.isTurning || isLeaning;

    // --- DETERMINE THREAT LEVEL ---
    if (myData.closestDistance <= 5.0f || (corneringDanger && myData.closestDistance <= 15.0f)) {
      threatLevel = 3; // CRITICAL: Very close, or cornering with a vehicle nearby
    } 
    else if (myData.closestDistance <= 10.0f) {
      threatLevel = 2; // WARNING: Standard danger zone
    } 
    else {
      threatLevel = 1; // CAUTION: Further out, but fast approaching (TTC triggered from Sender)
    }

    Serial.printf(">>> ALERT! Threat Level: %d | Closest Target: %.2f m\n", threatLevel, myData.closestDistance);
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

  // Failsafe: Turn off if connection lost OR if vibration lifespan expires
  if ((now - lastRecvTime > 2000) || (now > motorTurnOffTime)) {
    threatLevel = 0;
    hapticState = false;
    digitalWrite(MOTOR_PIN, LOW);
  } else {
    
    // --- HAPTIC PATTERN EXECUTION ---
    if (threatLevel == 3) {
      // CRITICAL: Solid, unbroken vibration
      digitalWrite(MOTOR_PIN, HIGH);
    } 
    else if (threatLevel == 2) {
      // WARNING: Fast Pulse (150ms ON / 150ms OFF)
      if (now - hapticPreviousMillis >= 150) {
        hapticPreviousMillis = now;
        hapticState = !hapticState;
        digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
      }
    } 
    else if (threatLevel == 1) {
      // CAUTION: Slow Pulse (400ms ON / 400ms OFF)
      if (now - hapticPreviousMillis >= 400) {
        hapticPreviousMillis = now;
        hapticState = !hapticState;
        digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
      }
    }
  }
  
  // A tiny delay yields time to the ESP32's background WiFi tasks
  delay(5); 
}
