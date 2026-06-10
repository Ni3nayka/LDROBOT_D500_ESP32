# LDROBOT_D500_ESP32

Arduino IDE library for reading LDROBOT D500 / STL-19P LiDAR packets with ESP32.

The parser runs in a background FreeRTOS task, reads the LiDAR UART stream and publishes the latest full 360 degree scan as angle and distance arrays.

## Hardware

- LiDAR: LDROBOT D500 / STL-19P
- Tested board: ESP32-S3 N16R8
- Arduino IDE board profile: ESP32S3 Dev Module
- Default LiDAR UART baud rate: 230400

Default examples use:

- LiDAR TX -> ESP32-S3 GPIO18
- LiDAR RX is not used by the library
- Common GND between ESP32 and LiDAR

Change `LIDAR_RX_PIN` in the examples if your LiDAR TX wire is connected to another ESP32 pin.

## Installation

1. Copy or clone this folder into your Arduino libraries directory.
2. Restart Arduino IDE.
3. Open `File > Examples > LDROBOT_D500_ESP32`.

## Examples

- `BasicRead` - direct example copied from `src/src.ino`.
- `SafeSnapshot` - copies the latest full scan with `copyData()` before printing points.
- `HealthMonitor` - prints parser counters, packet age and basic online status.

## Minimal Usage

```cpp
#include <LDROBOT_D500_ESP32.h>

HardwareSerial LidarUart(1);
LDROBOT_D500_ESP32 lidar;

constexpr int LIDAR_RX_PIN = 18;
constexpr int LIDAR_TX_PIN = -1;

void setup() {
  Serial.begin(115200);
  lidar.begin(LidarUart, LIDAR_RX_PIN, LIDAR_TX_PIN);
}

void loop() {
  Serial.println(lidar.getDataSize());
  delay(1000);
}
```

## Manuals

- https://www.waveshare.com/wiki/D500_LiDAR_Kit
- https://files.waveshare.com/wiki/D500-LiDAR-Kit/LDROBOT_STL-19P_Datasheet_EN_v1.0(1).pdf
