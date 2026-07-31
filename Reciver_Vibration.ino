#include <esp_now.h>
#include <WiFi.h>

#define VIBRATOR_PIN 2

typedef struct struct_message {
    float pitch;
    float roll;
    int sonar;
    int tof;
} struct_message;

struct_message receivedData;

unsigned long vibStartMillis = 0;
bool isVibrating = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    // Debug output: Verify this shows numbers like 40, 50, etc.
    Serial.printf("DEBUG: Sonar:%d, ToF:%d\n", receivedData.sonar, receivedData.tof);

    if (receivedData.sonar <= 50 && receivedData.tof <= 50) {
      if (!isVibrating) {
        digitalWrite(VIBRATOR_PIN, HIGH);
        vibStartMillis = millis();
        isVibrating = true;
      }
    }
  } else {
    Serial.printf("ERROR: Packet size mismatch! Got %d bytes\n", len);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(VIBRATOR_PIN, OUTPUT);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Receiver Ready.");
}

void loop() {
  if (isVibrating && (millis() - vibStartMillis >= 500)) {
    digitalWrite(VIBRATOR_PIN, LOW);
    isVibrating = false;
  }
  
}