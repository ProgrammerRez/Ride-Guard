#include <WiFi.h>
#include <esp_wifi.h>

#define VIBRATOR_PIN 2

void setup() {
  Serial.begin(115200);
  pinMode(VIBRATOR_PIN, OUTPUT);
  
  // Set to Station mode to access the radio hardware
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  
  if (ret == ESP_OK) {
    Serial.println();
    Serial.print("ESP32 Board MAC Address: ");
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", 
                  baseMac[0], baseMac[1], baseMac[2], 
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");

  }
  // Test Active HIGH
  digitalWrite(VIBRATOR_PIN, HIGH);
  delay(1000);
  digitalWrite(VIBRATOR_PIN, LOW);
  delay(1000);
}


void loop() {
  // Empty
}