#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

// ── CHANGE THESE TWO LINES PER SENSOR ─────────
const char* SENSOR_LABEL = "L_UA";
uint8_t HUB_MAC[] = {0xE8, 0x3D, 0xC1, 0x9C, 0x50, 0x14};
// ──────────────────────────────────────────────

const int LED_RED   = 1;
const int LED_GREEN = 0;

MPU6050 mpu;
uint8_t fifoBuffer[64];
esp_now_peer_info_t peerInfo;

#pragma pack(1)
struct SensorData {
  char label[8];
  float qw, qx, qy, qz;
};
#pragma pack()

SensorData data;

void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);

  // Power-on blink (5.5 s alternating R/G)
  for (int i = 0; i < 11; i++) {
    digitalWrite(LED_RED,   (i % 2) ? HIGH : LOW);
    digitalWrite(LED_GREEN, (i % 2) ? LOW  : HIGH);
    delay(500);
  }
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);

  mpu.initialize();
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    // Rapid red blink = fatal error
    while (1) {
      digitalWrite(LED_RED, LOW);  delay(100);
      digitalWrite(LED_RED, HIGH); delay(100);
    }
  }

  esp_now_register_send_cb(onSend);

  memcpy(peerInfo.peer_addr, HUB_MAC, 6);
  peerInfo.channel = 3;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1) {
      digitalWrite(LED_RED, LOW);  delay(100);
      digitalWrite(LED_RED, HIGH); delay(100);
    }
  }

  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
  } else {
    Serial.println("DMP init failed");
    while (1) {
      digitalWrite(LED_RED, LOW);  delay(100);
      digitalWrite(LED_RED, HIGH); delay(100);
    }
  }

  strncpy(data.label, SENSOR_LABEL, 7);
  data.label[7] = 0;

  // Green on 2 s = ready
  digitalWrite(LED_GREEN, LOW);
  delay(2000);
  digitalWrite(LED_GREEN, HIGH);

  Serial.println("Sensor ready: " + String(SENSOR_LABEL));
}

void loop() {
  static unsigned long lastSend  = 0;
  static unsigned long lastHB    = 0;
  static bool          hbOn      = false;
  static unsigned long hbStart   = 0;

  unsigned long now = millis();

  // Red heartbeat (150 ms pulse every 1 s)
  if (!hbOn && now - lastHB >= 1000) {
    digitalWrite(LED_RED, LOW);
    hbOn    = true;
    hbStart = now;
    lastHB  = now;
  }
  if (hbOn && now - hbStart >= 150) {
    digitalWrite(LED_RED, HIGH);
    hbOn = false;
  }

  if (now - lastSend < 15) return;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  lastSend = now;
  Quaternion q;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  data.qw = q.w;
  data.qx = q.x;
  data.qy = q.y;
  data.qz = q.z;

  esp_now_send(HUB_MAC, (uint8_t*)&data, sizeof(data));
}