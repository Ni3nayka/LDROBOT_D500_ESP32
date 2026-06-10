#include <Arduino.h>
#include <LDROBOT_D500_ESP32.h>

HardwareSerial LidarUart(1);
LDROBOT_D500_ESP32 lidar;

constexpr int LIDAR_RX_PIN = 18;
constexpr int LIDAR_TX_PIN = -1;
constexpr uint32_t LIDAR_TIMEOUT_MS = 1000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("LDROBOT D500 health monitor example");

  if (lidar.begin(LidarUart, LIDAR_RX_PIN, LIDAR_TX_PIN)) {
    Serial.println("Lidar parser started");
  } else {
    Serial.println("Lidar parser start FAILED");
  }
}

void loop() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint < 1000) {
    delay(5);
    return;
  }

  lastPrint = millis();

  uint32_t packetAge = millis() - lidar.lastPacketMs;
  bool online = lidar.lastPacketMs > 0 && packetAge <= LIDAR_TIMEOUT_MS;

  Serial.printf(
    "online=%s, packetAge=%lu ms, speedRaw=%d, frames=%lu, rotations=%lu, crcErr=%lu, syncErr=%lu, overflow=%lu, points=%d\n",
    online ? "yes" : "no",
    (unsigned long)packetAge,
    lidar.lastSpeedRaw,
    (unsigned long)lidar.frameCount,
    (unsigned long)lidar.rotationCount,
    (unsigned long)lidar.crcErrorCount,
    (unsigned long)lidar.syncErrorCount,
    (unsigned long)lidar.overflowCount,
    lidar.getDataSize()
  );
}
