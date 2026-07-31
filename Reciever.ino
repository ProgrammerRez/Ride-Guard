#include <esp_now.h>
#include <WiFi.h>

// Replace with your OWN MAC address for this specific board
uint8_t myMac[] = {0x68, 0x25, 0xDD, 0x22, 0x9B, 0x14}; 

uint8_t peers[][6] = {
  {0x68, 0x25, 0xDD, 0x22, 0x9B, 0x14},
  {0x28, 0x05, 0xA5, 0x2D, 0x1A, 0x88},
  {0x8C, 0x94, 0xDF, 0x6D, 0xB1, 0xE8}
};

typedef struct struct_message {
    char sender[18];
    int cmd;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("[RECV] Command %d from MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                myData.cmd, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Essential for Serial Monitor to catch the first print
  
  WiFi.mode(WIFI_STA);
  Serial.printf("Board MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW Init Failed");
    return;
  }
  Serial.println("[INFO] ESP-NOW Init Success");

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peers
  for (int i = 0; i < 3; i++) {
    // Skip adding our own MAC as a peer
    if (memcmp(peers[i], myMac, 6) == 0) continue;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peers[i], 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.printf("[ERROR] Failed to add peer: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                    peers[i][0], peers[i][1], peers[i][2], peers[i][3], peers[i][4], peers[i][5]);
    } else {
      Serial.printf("[INFO] Peer added: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                    peers[i][0], peers[i][1], peers[i][2], peers[i][3], peers[i][4], peers[i][5]);
    }
  }
}

void loop() {
  // Add loop logic to broadcast commands here
  delay(5000);
}