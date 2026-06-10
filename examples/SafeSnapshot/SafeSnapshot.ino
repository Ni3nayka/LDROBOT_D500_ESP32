#include <Arduino.h>
#include <LDROBOT_D500_ESP32.h>

HardwareSerial LidarUart(1);
LDROBOT_D500_ESP32 lidar;

constexpr int LIDAR_RX_PIN = 18;
constexpr int LIDAR_TX_PIN = -1;

float angles[LDROBOT_D500_ESP32::DATA_CAPACITY];
int distances[LDROBOT_D500_ESP32::DATA_CAPACITY];
int scanSize = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("LDROBOT D500 safe snapshot example");

  if (!lidar.begin(LidarUart, LIDAR_RX_PIN, LIDAR_TX_PIN)) {
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
  lidar.copyData(angles, distances, scanSize);

  Serial.printf("scanSize=%d, rotations=%lu\n", scanSize, (unsigned long)lidar.rotationCount);

  if (scanSize <= 0) {
    Serial.println("Waiting for a full scan...");
    return;
  }

  int samplesToPrint = scanSize < 10 ? scanSize : 10;
  int step = scanSize / samplesToPrint;
  if (step <= 0) {
    step = 1;
  }

  for (int i = 0; i < scanSize && samplesToPrint > 0; i += step, samplesToPrint--) {
    Serial.printf("%d: angle=%.2f deg, distance=%d mm\n", i, angles[i], distances[i]);
  }

  Serial.println();
}
