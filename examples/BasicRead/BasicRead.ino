#include <Arduino.h>
#include "LDROBOT_D500_ESP32.h"

HardwareSerial LidarUart(1);
LDROBOT_D500_ESP32 lidar;

// TX лидара -> GPIO48 ESP32-S3
constexpr int LIDAR_RX_PIN = 18;
constexpr int LIDAR_TX_PIN = -1;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-S3 + LDRobot D500 / STL-19P");

  bool ok = lidar.begin(
    LidarUart,
    LIDAR_RX_PIN,
    LIDAR_TX_PIN
  );

  if (ok) {
    Serial.println("Lidar parser started");
  } else {
    Serial.println("Lidar parser start FAILED");
  }
}

void loop() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.printf(
      "frames=%lu, rotations=%lu, crcErr=%lu, syncErr=%lu, overflow=%lu, data_size=%d\n",
      (unsigned long)lidar.frameCount,
      (unsigned long)lidar.rotationCount,
      (unsigned long)lidar.crcErrorCount,
      (unsigned long)lidar.syncErrorCount,
      (unsigned long)lidar.overflowCount,
      lidar.data_size
    );

    // Пример вывода первых 10 точек последнего полного оборота
    int count = lidar.data_size;
    if (count > 10) count = 10;

    for (int i = 0; i < lidar.data_size; i+=lidar.data_size/10) {
      Serial.printf(
        "%d: angle=%.2f deg, distance=%d mm\n",
        i,
        lidar.data_angle[i],
        lidar.data_distance[i]
      );
    }

    Serial.println();
  }

  delay(5);
}