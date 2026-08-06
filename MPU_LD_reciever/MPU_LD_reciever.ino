/*
  Device 2: ESP-NOW Receiver + Haptic Language System
  Structured Layout: Variables -> Functions -> Haptics -> Output
*/

#include <esp_now.h>
#include <WiFi.h>


// ==========================================
// 1. RECEIVER VARIABLES
// ==========================================

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
    float turnRate; // Adjusted from gyroZ to match Sender
    bool isTurning;
    uint8_t targetCount;
    Target targets[8];
} struct_message;

struct_message myData;

// Timing and State Variables
unsigned long lastRecvTime = 0;
unsigned long motorTurnOffTime = 0;
const unsigned long VIBRATION_LIFESPAN_MS = 1500; 
unsigned long lastPrintTime = 0;

// Haptic Pattern Variables
uint8_t threatLevel = 0; // 0=Off, 1=Awareness, 2=Danger, 3=Cooked
unsigned long hapticPreviousMillis = 0;
bool hapticState = false;


// ==========================================
// 2. RECEIVER FUNCTIONS
// ==========================================

// ESP-NOW Callback
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();

  if (myData.vibrate && myData.closestDistance > 0.0f) {
    motorTurnOffTime = millis() + VIBRATION_LIFESPAN_MS;
    
    // Evaluate leaning/turning vulnerability
    bool isLeaning = (abs(myData.roll) > 20.0f) || (abs(myData.pitch) > 40.0f);
    bool corneringDanger = myData.isTurning || isLeaning;

    // Determine Threat Level based on ranges
    if (myData.closestDistance <= 5.0f || (corneringDanger && myData.closestDistance <= 15.0f)) {
      threatLevel = 3; // COOKED: Very close or mid-corner threat
    } 
    else if (myData.closestDistance <= 10.0f) {
      threatLevel = 2; // DANGER: 5m to 10m
    } 
    else {
      threatLevel = 1; // AWARENESS: > 10m (Triggered by TTC/speed)
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


// ==========================================
// 3. HAPTICS LOGIC FUNCTION
// ==========================================

void processHaptics(unsigned long now) {
  // Failsafe: Turn off if connection lost OR if vibration lifespan expires
  if ((now - lastRecvTime > 2000) || (now > motorTurnOffTime)) {
    threatLevel = 0;
    hapticState = false;
    digitalWrite(MOTOR_PIN, LOW);
    return;
  }

  // Handle specific threat levels dynamically
  if (threatLevel == 3) {
    // LEVEL 3: COOKED - Continuous unbroken vibration
    digitalWrite(MOTOR_PIN, HIGH);
  } 
  else if (threatLevel == 2) {
    // LEVEL 2: DANGER - Small dynamic bursts (Distance: 5m -> 10m)
    // Maps 5.0m to 100ms (very fast) and 10.0m to 300ms (moderately fast)
    long distanceInt = constrain((long)(myData.closestDistance * 10), 50, 100);
    long dynamicPulse = map(distanceInt, 50, 100, 100, 300);

    if (now - hapticPreviousMillis >= dynamicPulse) {
      hapticPreviousMillis = now;
      hapticState = !hapticState;
      digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
    }
  } 
  else if (threatLevel == 1) {
    // LEVEL 1: AWARENESS - Long dynamic bursts (Distance: 10m -> 25m+)
    // Maps 10.0m to 400ms (medium pulse) and 25.0m to 800ms (slow pulse)
    long distanceInt = constrain((long)(myData.closestDistance * 10), 100, 250);
    long dynamicPulse = map(distanceInt, 100, 250, 400, 800);

    if (now - hapticPreviousMillis >= dynamicPulse) {
      hapticPreviousMillis = now;
      hapticState = !hapticState;
      digitalWrite(MOTOR_PIN, hapticState ? HIGH : LOW);
    }
  }
}


// ==========================================
// 4. PRINT DATA (DELAY 500MS)
// ==========================================

void printTelemetry(unsigned long now) {
  // Uses a non-blocking timer to print every 500ms so haptics aren't interrupted
  if (now - lastPrintTime >= 500) {
    lastPrintTime = now;
    
    Serial.printf("[Telemetry] Pitch: %.1f° | Roll: %.1f° | TurnRate: %.1f°/s | Turning: %s\n", 
                  myData.pitch, myData.roll, myData.turnRate, myData.isTurning ? "YES" : "NO");

    if (threatLevel > 0) {
      Serial.printf(">>> ALERT! Threat Level: %d | Closest Target: %.2f m | Targets tracked: %u\n", 
                    threatLevel, myData.closestDistance, myData.targetCount);
    }
  }
}


// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  unsigned long currentMillis = millis();

  processHaptics(currentMillis);
  printTelemetry(currentMillis);
  
  // A tiny delay yields time to the ESP32's background WiFi tasks
  delay(5); 
}
