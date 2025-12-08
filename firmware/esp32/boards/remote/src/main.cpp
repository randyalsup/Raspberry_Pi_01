#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "pin_config.h"

// WiFi credentials for OTA updates
const char* ssid = "Alsupnet";
const char* password = "Asil1962";

// ========================================
// ESP32 Remote Gateway (RC)
// Purpose: UART (Pi Zero 2W) <-> ESP-NOW (Robot ESP32) Bridge
// ========================================

// Robot ESP32 MAC address
uint8_t robotMAC[] = {0xEC, 0xE3, 0x34, 0x22, 0xD9, 0xB0}; // RO device
// This device MAC: 80:F3:DA:42:CC:A0

// UART Protocol constants
#define SYNC_BYTE 0xAA
#define END_BYTE  0x55
#define MAX_PAYLOAD 250

// Message buffer
struct UARTMessage {
  uint8_t type;
  uint8_t length;
  uint8_t payload[MAX_PAYLOAD];
};

bool robotPaired = false;

// Forward declarations
void forwardToRobot(UARTMessage *msg);

// ========================================
// CRC-8
// ========================================
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
  }
  return crc;
}

// ========================================
// UART → ESP-NOW
// ========================================
void processUARTFromPi() {
  static uint8_t state = 0;
  static UARTMessage msg;
  static uint8_t idx = 0;
  
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    
    switch(state) {
      case 0: if (b == SYNC_BYTE) state = 1; break;
      case 1: msg.type = b; state = 2; break;
      case 2: msg.length = b; idx = 0; state = (msg.length > 0) ? 3 : 4; break;
      case 3: 
        if (idx < MAX_PAYLOAD && idx < msg.length) msg.payload[idx++] = b;
        if (idx >= msg.length) state = 4;
        break;
      case 4: {
        uint8_t data[2 + MAX_PAYLOAD];
        data[0] = msg.type; data[1] = msg.length;
        memcpy(&data[2], msg.payload, msg.length);
        if (crc8(data, 2 + msg.length) == b) state = 5;
        else state = 0;
        break;
      }
      case 5:
        if (b == END_BYTE) forwardToRobot(&msg);
        state = 0;
        break;
    }
  }
}

void forwardToRobot(UARTMessage *msg) {
  if (!robotPaired) return;
  uint8_t packet[2 + MAX_PAYLOAD];
  packet[0] = msg->type;
  packet[1] = msg->length;
  memcpy(&packet[2], msg->payload, msg->length);
  esp_now_send(robotMAC, packet, 2 + msg->length);
}

// ========================================
// ESP-NOW → UART
// ========================================
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 2) return;
  
  uint8_t msgType = data[0];
  uint8_t msgLen = data[1];
  if (msgLen != len - 2) return;
  
  uint8_t frame[5 + MAX_PAYLOAD];
  frame[0] = SYNC_BYTE;
  frame[1] = msgType;
  frame[2] = msgLen;
  memcpy(&frame[3], &data[2], msgLen);
  
  uint8_t crcData[2 + MAX_PAYLOAD];
  crcData[0] = msgType;
  crcData[1] = msgLen;
  memcpy(&crcData[2], &data[2], msgLen);
  frame[3 + msgLen] = crc8(crcData, 2 + msgLen);
  frame[4 + msgLen] = END_BYTE;
  
  Serial2.write(frame, 5 + msgLen);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Silent
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(PI_BAUD_RATE, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  delay(500);
  
  Serial.println("\n=== ESP32 Remote Gateway (RC) ===");
  Serial.println("UART <-> ESP-NOW Bridge");
  
  // Connect to WiFi for OTA
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed, continuing without OTA");
  }
  
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  
  // Setup OTA
  ArduinoOTA.setHostname("ESP32-Remote");
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, robotMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    robotPaired = true;
    Serial.println("Paired with robot");
  }
  
  Serial.println("Ready");
}

// ========================================
// Loop
// ========================================
void loop() {
  ArduinoOTA.handle();
  processUARTFromPi();
  
  // TODO: UWB ranging when module arrives
  
  delay(1);
}
