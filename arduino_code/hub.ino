#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

const char* SELF_LABEL = "HIPS";
const char* ssid       = "TP-Link_DF6C_Cave";
const char* password   = "Caveiot@123";

const char*  multicast_ip = "239.0.0.1";
const unsigned int port   = 5005;

const int LED_RED   = 1;
const int LED_GREEN = 0;

WiFiUDP udp;
MPU6050 mpu;
uint8_t fifoBuffer[64];

#pragma pack(1)
struct SensorData {
  char label[8];
  float qw, qx, qy, qz;
};
#pragma pack()

static SensorData bufA[8], bufB[8];
static volatile SensorData* writeBuf = bufA;
static SensorData* readBuf = bufB;
static volatile int writeCount = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void onEspNowReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(SensorData)) return;
  portENTER_CRITICAL_ISR(&mux);
  if (writeCount < 8) {
    memcpy((void*)&writeBuf[writeCount], incomingData, sizeof(SensorData));
    writeCount++;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void forwardPacket(const char* label, float qw, float qx, float qy, float qz) {
  char payload[64];
  snprintf(payload, sizeof(payload), "%s,%.4f,%.4f,%.4f,%.4f", label, qw, qx, qy, qz);
  udp.beginPacket(multicast_ip, port);
  udp.print(payload);
  udp.endPacket();
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_RED, LOW);   delay(100);
    digitalWrite(LED_RED, HIGH);  delay(100);
    if (millis() - start > 10000) return;
  }
}

void broadcastChannel() {
  // Blink green N times = WiFi channel number, so you know without serial
  uint8_t ch = WiFi.channel();
  delay(1000);
  for (uint8_t i = 0; i < ch; i++) {
    digitalWrite(LED_GREEN, LOW);  delay(200);
    digitalWrite(LED_GREEN, HIGH); delay(200);
  }
  delay(1000);
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

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_GREEN, LOW);
    delay(2000);
    digitalWrite(LED_GREEN, HIGH);
    // Blink green to show channel — count the blinks
    broadcastChannel();
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

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_GREEN, HIGH);
    connectWiFi();
    return;
  }

  if (!hbOn && now - lastHB >= 1000) {
    digitalWrite(LED_RED, LOW);
    hbOn = true; hbStart = now; lastHB = now;
  }
  if (hbOn && now - hbStart >= 150) {
    digitalWrite(LED_RED, HIGH);
    hbOn = false;
  }

  // Own MPU first to avoid FIFO overflow
  if (now - lastOwnSend >= 15) {
    if (mpu.getFIFOCount() >= 1024) {
      mpu.resetFIFO();
    } else if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      lastOwnSend = now;
      Quaternion q;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      forwardPacket(SELF_LABEL, q.w, q.x, q.y, q.z);
    }
  }

  // Swap buffers atomically
  int count = 0;
  portENTER_CRITICAL(&mux);
  count = writeCount;
  writeCount = 0;
  volatile SensorData* tmp = writeBuf;
  writeBuf = (volatile SensorData*)readBuf;
  readBuf = (SensorData*)tmp;
  portEXIT_CRITICAL(&mux);

  for (int i = 0; i < count; i++) {
    forwardPacket(readBuf[i].label, readBuf[i].qw, readBuf[i].qx, readBuf[i].qy, readBuf[i].qz);
  }

  delay(1);
}