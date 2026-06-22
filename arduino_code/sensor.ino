#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

const char* SENSOR_LABEL = "L_UA";
uint8_t HUB_MAC[] = {0xE8, 0x3D, 0xC1, 0x9C, 0x50, 0x14};

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

uint8_t scanForHubChannel() {
  WiFi.mode(WIFI_STA);
  for (uint8_t ch = 1; ch <= 13; ch++) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() == ESP_OK) {
      memcpy(peerInfo.peer_addr, HUB_MAC, 6);
      peerInfo.channel = ch;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        SensorData dummy = {};
        strncpy(dummy.label, "PING", 7);
        esp_err_t result = esp_now_send(HUB_MAC, (uint8_t*)&dummy, sizeof(dummy));
        delay(50);
        esp_now_del_peer(HUB_MAC);  // correct function name
        esp_now_deinit();
        if (result == ESP_OK) return ch;
      } else {
        esp_now_deinit();
      }
    }
  }
  return 3;
}

void setup() {
  Wire.begin(8, 9);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);

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

  uint8_t ch = scanForHubChannel();

  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    while (1) { digitalWrite(LED_RED, LOW); delay(100); digitalWrite(LED_RED, HIGH); delay(100); }
  }

  esp_now_register_send_cb(onSend);

  memcpy(peerInfo.peer_addr, HUB_MAC, 6);
  peerInfo.channel = ch;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    while (1) { digitalWrite(LED_RED, LOW); delay(100); digitalWrite(LED_RED, HIGH); delay(100); }
  }

  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
  } else {
    while (1) { digitalWrite(LED_RED, LOW); delay(100); digitalWrite(LED_RED, HIGH); delay(100); }
  }

  strncpy(data.label, SENSOR_LABEL, 7);
  data.label[7] = 0;

  digitalWrite(LED_GREEN, LOW);
  delay(2000);
  digitalWrite(LED_GREEN, HIGH);
}

void loop() {
  static unsigned long lastSend  = 0;
  static unsigned long lastHB    = 0;
  static bool          hbOn      = false;
  static unsigned long hbStart   = 0;

  unsigned long now = millis();

  if (!hbOn && now - lastHB >= 1000) {
    digitalWrite(LED_RED, LOW);
    hbOn = true; hbStart = now; lastHB = now;
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
  data.qw = q.w; data.qx = q.x; data.qy = q.y; data.qz = q.z;

  esp_now_send(HUB_MAC, (uint8_t*)&data, sizeof(data));
}