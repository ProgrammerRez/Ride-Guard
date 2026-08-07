/*
  Device 2: ESP-NOW Receiver + Haptic Language System
  Structured Layout: Variables -> Functions -> Haptics -> Output
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

typedef struct struct_message {
    bool vibrate;
    float closestDistance;
    float pitch;
    float roll;
    float turnRate; 
    bool isTurning;
    uint8_t targetCount;
    Target targets[8];
} struct_message;

struct_message myData;

unsigned long lastRecvTime = 0;
unsigned long motorTurnOffTime = 0;
const unsigned long VIBRATION_LIFESPAN_MS = 800; // Optimized response window
unsigned long lastPrintTime = 0;

uint8_t threatLevel = 0; 
unsigned long hapticPreviousMillis = 0;
bool hapticState = false;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();

  if (myData.vibrate && myData.closestDistance > 0.0f) {
    motorTurnOffTime = millis() + VIBRATION_LIFESPAN_MS;
    
    // Validated leans/turns based on zeroed plane metrics
    bool isLeaning = (abs(myData.roll) > 20.0f) || (abs(myData.pitch) > 35.0f);
    bool corneringDanger = myData.isTurning || isLeaning;

    if (myData.closestDistance <= 3.0f || (corneringDanger && myData.closestDistance <= 15.0f)) {
      threatLevel = 3; // COOKED
    } 
    else if (myData.closestDistance <= 5.0f) {
      threatLevel = 2; // DANGER
    } 
    else if (myData.closestDistance <= 15.0f) {
      threatLevel = 1; // AWARENESS
    }
    else {
      threatLevel = 0; 
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void processHaptics(unsigned long now) {
  if ((now - lastRecvTime > 2000) || (now > motorTurnOffTime) || threatLevel == 0) {
    threatLevel = 0;
    hapticState = false;
    digitalWrite(MOTOR_PIN, LOW);
    return;
  }

  if (threatLevel == 3) {
    digitalWrite(MOTOR_PIN, HIGH);
  } 
  else if (threatLevel == 2) {
    long distanceInt = constrain((long)(myData.closestDistance * 10), 30, 50);
    long dynamicPulse = map(distanceInt, 30, 50, 100, 300);

    if (now - hapticPreviousMillis >= dynamicPulse) {
      hapticPreviousMillis = now;
      hapticState = !hapticState;
      digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
    }
  } 
  else if (threatLevel == 1) {
    long distanceInt = constrain((long)(myData.closestDistance * 10), 50, 150);
    long dynamicPulse = map(distanceInt, 50, 150, 400, 800);

    if (now - hapticPreviousMillis >= dynamicPulse) {
      hapticPreviousMillis = now;
      hapticState = !hapticState;
      digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
    }
  }
}

void printTelemetry(unsigned long now) {
  if (now - lastPrintTime >= 500) {
    lastPrintTime = now;
    
    Serial.printf("[Telemetry] Pitch: %.1f° | Roll: %.1f° | TurnRate: %.1f°/s | Turning: %s\n", 
                  myData.pitch, myData.roll, myData.turnRate, myData.isTurning ? "YES" : "NO");

    if (threatLevel > 0 && myData.targetCount > 0) {
      Serial.printf(">>> ALERT! Threat Level: %d | Closest Target: %.2f m | Targets tracked: %u\n", 
                    threatLevel, myData.closestDistance, myData.targetCount);
    }
  }
}

void loop() {
  unsigned long currentMillis = millis();

  processHaptics(currentMillis);
  printTelemetry(currentMillis);
  delay(5); 
}
