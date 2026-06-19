#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

// ── Change per hub ──────────────────────────
const char* SELF_LABEL = "HIPS";                // "HIPS" for Waist
const char* ssid       = "TP-Link_DF6C_Cave";
const char* password   = "Caveiot@123";
// ─────────────────────────────────────────────

const char*  multicast_ip = "239.0.0.1";
const unsigned int port   = 5005;

// ── LED pins (active‑low) ────────────────────
const int LED_RED   = 1;
const int LED_GREEN = 0;

WiFiUDP udp;
MPU6050 mpu;
uint8_t fifoBuffer[64];

// ── ESP‑Now packet format ────────────────────
#pragma pack(1)
struct SensorData {
  char label[8];
  float qw, qx, qy, qz;
};
#pragma pack()

// ── Simple ring‑buffer queue (max 8 pending) ──
volatile int pendingCount = 0;
SensorData pending[8];

// Called from ESP‑Now interrupt – just store
void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(SensorData)) return;
  if (pendingCount >= 8) return;                      // drop if queue full
  memcpy((void*)&pending[pendingCount], incomingData, sizeof(SensorData));
  pendingCount++;
}

// ── Safe UDP sender (called only from loop) ──
void forwardPacket(const char* label, float qw, float qx, float qy, float qz) {
  char payload[64];
  snprintf(payload, sizeof(payload), "%s,%.4f,%.4f,%.4f,%.4f", label, qw, qx, qy, qz);
  udp.beginPacket(multicast_ip, port);
  udp.print(payload);
  udp.endPacket();
}

// ── WiFi reconnect helper ────────────────────
void connectWiFi() {
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_RED, LOW);   delay(100);
    digitalWrite(LED_RED, HIGH);  delay(100);
    if (millis() - start > 10000) return;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);

  // Power‑on blink (5.5 s alternating R/G)
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

  connectWiFi();

  // WiFi OK → green 2 s
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_GREEN, LOW);
    delay(2000);
    digitalWrite(LED_GREEN, HIGH);
  }

  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
  } else {
    while (1) {
      digitalWrite(LED_RED, LOW);   delay(100);
      digitalWrite(LED_RED, HIGH);  delay(100);
    }
  }

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onEspNowReceive);

  udp.begin(multicast_ip, port);
}

void loop() {
  static unsigned long lastOwnSend = 0;
  static unsigned long lastHB = 0;
  static bool hbOn = false;
  static unsigned long hbStart = 0;

  unsigned long now = millis();

  // ── WiFi reconnect watchdog ──────────────
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_GREEN, HIGH);
    connectWiFi();
    return;
  }

  // ── Red heartbeat (150 ms every 1 s) ─────
  if (!hbOn && now - lastHB >= 1000) {
    digitalWrite(LED_RED, LOW);
    hbOn = true;
    hbStart = now;
    lastHB = now;
  }
  if (hbOn && now - hbStart >= 150) {
    digitalWrite(LED_RED, HIGH);
    hbOn = false;
  }

  // ── Forward any pending limb data ─────────
  int count = pendingCount;
  pendingCount = 0;
  for (int i = 0; i < count; i++) {
      forwardPacket(pending[i].label, pending[i].qw, pending[i].qx, pending[i].qy, pending[i].qz);
  }

  // ── Read & send own sensor (~66 Hz) ──────
  if (now - lastOwnSend >= 15) {
    if (mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO();
      return;
    }
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      lastOwnSend = now;
      Quaternion q;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      forwardPacket(SELF_LABEL, q.w, q.x, q.y, q.z);
    }
  }

  delay(1);
}