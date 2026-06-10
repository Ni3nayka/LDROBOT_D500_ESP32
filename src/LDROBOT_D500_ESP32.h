#ifndef LDROBOT_D500_ESP32_H
#define LDROBOT_D500_ESP32_H

#include <Arduino.h>

class LDROBOT_D500_ESP32 {
public:
  // D500/STL-19P:
  // до 5000 измерений/с.
  // Минимальная частота вращения, которую берём для расчёта запаса: 6 Гц.
  //
  // 5000 / 6 = 833.33 -> 834 точки максимум на оборот.
  // +20% запаса -> 1001 точка.
  static constexpr int MAX_RANGING_FREQUENCY_HZ = 5000;
  static constexpr int MIN_SCANNING_FREQUENCY_HZ = 6;

  static constexpr int MAX_POINTS_PER_SCAN =
    (MAX_RANGING_FREQUENCY_HZ + MIN_SCANNING_FREQUENCY_HZ - 1) /
    MIN_SCANNING_FREQUENCY_HZ;

  static constexpr int DATA_CAPACITY =
    (MAX_POINTS_PER_SCAN * 120 + 99) / 100;

  // Главные массивы данных.
  //
  // data_angle[i]    = реальный угол точки, градусы, float
  // data_distance[i] = расстояние до точки, мм
  // data_size        = количество занятых ячеек
  //
  // В этих массивах лежит последний полностью собранный оборот лидара.
  volatile float data_angle[DATA_CAPACITY];
  volatile int data_distance[DATA_CAPACITY];
  volatile int data_size = 0;

  volatile uint32_t frameCount = 0;
  volatile uint32_t rotationCount = 0;
  volatile uint32_t crcErrorCount = 0;
  volatile uint32_t syncErrorCount = 0;
  volatile uint32_t overflowCount = 0;
  volatile uint32_t lastPacketMs = 0;

  volatile int lastSpeedRaw = 0;

  LDROBOT_D500_ESP32() {
    initPublicData();
    initWorkData();
  }

  bool begin(
    HardwareSerial &serialPort,
    int rxPin,
    int txPin = -1,
    uint32_t baud = 230400,
    uint32_t rxBufferSize = 8192,
    uint32_t taskStackSize = 4096,
    UBaseType_t taskPriority = 2,
    BaseType_t taskCore = 1
  ) {
    if (_running) {
      return true;
    }

    _serial = &serialPort;
    _rxPin = rxPin;
    _txPin = txPin;
    _baud = baud;

    if (_dataMutex == nullptr) {
      _dataMutex = xSemaphoreCreateMutex();
    }

    initPublicData();
    initWorkData();

    _serial->setRxBufferSize(rxBufferSize);

    _serial->begin(
      _baud,
      SERIAL_8N1,
      _rxPin,
      _txPin
    );

    while (_serial->available()) {
      _serial->read();
    }

    _running = true;

    BaseType_t result = xTaskCreatePinnedToCore(
      taskTrampoline,
      "D500_Lidar_Task",
      taskStackSize,
      this,
      taskPriority,
      &_taskHandle,
      taskCore
    );

    if (result != pdPASS) {
      _running = false;
      _taskHandle = nullptr;
      return false;
    }

    return true;
  }

  void end() {
    _running = false;

    if (_taskHandle != nullptr) {
      vTaskDelay(pdMS_TO_TICKS(10));
      vTaskDelete(_taskHandle);
      _taskHandle = nullptr;
    }

    if (_serial != nullptr) {
      _serial->end();
    }
  }

  bool isRunning() const {
    return _running;
  }

  int getDataSize() {
    int size = 0;

    lockData();
    size = data_size;
    unlockData();

    return size;
  }

  float getAngle(int index) {
    float value = 0.0f;

    if (index < 0 || index >= DATA_CAPACITY) {
      return 0.0f;
    }

    lockData();
    if (index < data_size) {
      value = data_angle[index];
    }
    unlockData();

    return value;
  }

  int getDistanceByIndex(int index) {
    int value = 0;

    if (index < 0 || index >= DATA_CAPACITY) {
      return 0;
    }

    lockData();
    if (index < data_size) {
      value = data_distance[index];
    }
    unlockData();

    return value;
  }

  // Безопасное копирование последнего полного скана.
  //
  // Использование:
  // float angles[LDROBOT_D500_ESP32::DATA_CAPACITY];
  // int distances[LDROBOT_D500_ESP32::DATA_CAPACITY];
  // int size = 0;
  //
  // lidar.copyData(angles, distances, size);
  void copyData(
    float destinationAngles[DATA_CAPACITY],
    int destinationDistances[DATA_CAPACITY],
    int &destinationSize
  ) {
    lockData();

    destinationSize = data_size;

    for (int i = 0; i < destinationSize; i++) {
      destinationAngles[i] = data_angle[i];
      destinationDistances[i] = data_distance[i];
    }

    unlockData();
  }

private:
  static constexpr uint8_t HEADER = 0x54;
  static constexpr uint8_t VER_LEN = 0x2C;

  static constexpr uint8_t POINTS_PER_FRAME = 12;
  static constexpr uint8_t FRAME_LENGTH = 47;

  HardwareSerial *_serial = nullptr;
  TaskHandle_t _taskHandle = nullptr;
  SemaphoreHandle_t _dataMutex = nullptr;

  volatile bool _running = false;

  int _rxPin = -1;
  int _txPin = -1;
  uint32_t _baud = 230400;

  uint8_t _frame[FRAME_LENGTH];
  uint8_t _frameIndex = 0;
  uint8_t _parserState = 0;

  // Рабочий буфер текущего оборота.
  // Именно сюда фоновая задача складывает новые точки.
  // Когда оборот завершён, буфер публикуется в data_angle/data_distance.
  float _workAngle[DATA_CAPACITY];
  int _workDistance[DATA_CAPACITY];
  int _workSize = 0;

  bool _hasLastWorkAngle = false;
  float _lastWorkAngle = 0.0f;

  static void taskTrampoline(void *param) {
    LDROBOT_D500_ESP32 *self = static_cast<LDROBOT_D500_ESP32 *>(param);

    if (self != nullptr) {
      self->taskLoop();
    }

    vTaskDelete(nullptr);
  }

  void taskLoop() {
    while (_running) {
      if (_serial == nullptr) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      bool hadData = false;

      while (_serial->available() > 0) {
        hadData = true;
        uint8_t b = static_cast<uint8_t>(_serial->read());
        parseByte(b);
      }

      if (!hadData) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }

  void parseByte(uint8_t b) {
    switch (_parserState) {
      case 0:
        if (b == HEADER) {
          _frame[0] = b;
          _frameIndex = 1;
          _parserState = 1;
        }
        break;

      case 1:
        if (b == VER_LEN) {
          _frame[1] = b;
          _frameIndex = 2;
          _parserState = 2;
        } else {
          syncErrorCount++;

          if (b == HEADER) {
            _frame[0] = b;
            _frameIndex = 1;
            _parserState = 1;
          } else {
            _frameIndex = 0;
            _parserState = 0;
          }
        }
        break;

      case 2:
        _frame[_frameIndex++] = b;

        if (_frameIndex >= FRAME_LENGTH) {
          handleFrame(_frame);
          _frameIndex = 0;
          _parserState = 0;
        }
        break;

      default:
        _frameIndex = 0;
        _parserState = 0;
        break;
    }
  }

  void handleFrame(const uint8_t *frame) {
    if (frame[0] != HEADER || frame[1] != VER_LEN) {
      syncErrorCount++;
      return;
    }

    uint8_t calculatedCrc = calcCRC8(frame, FRAME_LENGTH - 1);
    uint8_t receivedCrc = frame[FRAME_LENGTH - 1];

    if (calculatedCrc != receivedCrc) {
      crcErrorCount++;
      return;
    }

    uint16_t speedRaw = readU16LE(&frame[2]);
    uint16_t startAngleRaw = readU16LE(&frame[4]) % 36000;
    uint16_t endAngleRaw = readU16LE(&frame[42]) % 36000;

    lastSpeedRaw = speedRaw;

    int32_t angleDiff =
      static_cast<int32_t>(endAngleRaw) -
      static_cast<int32_t>(startAngleRaw);

    // Если кадр пересекает 0 градусов
    if (angleDiff < 0) {
      angleDiff += 36000;
    }

    for (uint8_t i = 0; i < POINTS_PER_FRAME; i++) {
      uint8_t pointOffset = 6 + i * 3;

      uint16_t distanceMm = readU16LE(&frame[pointOffset]);

      float angleRaw =
        static_cast<float>(startAngleRaw) +
        (static_cast<float>(angleDiff) * static_cast<float>(i)) /
        static_cast<float>(POINTS_PER_FRAME - 1);

      float angleDeg = angleRaw / 100.0f;

      if (angleDeg >= 360.0f) {
        angleDeg -= 360.0f;
      }

      addPointToWorkScan(angleDeg, static_cast<int>(distanceMm));
    }

    frameCount++;
    lastPacketMs = millis();
  }

  void addPointToWorkScan(float angleDeg, int distanceMm) {
    // Детект нового оборота.
    //
    // Когда угол резко переходит, например:
    // 359.4 -> 0.2,
    // значит предыдущий оборот завершён.
    if (_hasLastWorkAngle) {
      bool crossedZero =
        (_lastWorkAngle > 300.0f) &&
        (angleDeg < 60.0f);

      if (crossedZero) {
        publishWorkScan();
        initWorkData();
        rotationCount++;
      }
    }

    _hasLastWorkAngle = true;
    _lastWorkAngle = angleDeg;

    if (_workSize < DATA_CAPACITY) {
      _workAngle[_workSize] = angleDeg;
      _workDistance[_workSize] = distanceMm;
      _workSize++;
    } else {
      overflowCount++;
    }
  }

  void publishWorkScan() {
    // Пустой или слишком маленький скан не публикуем.
    // Это защищает от случайного мусора при старте.
    if (_workSize <= 0) {
      return;
    }

    lockData();

    data_size = _workSize;

    if (data_size > DATA_CAPACITY) {
      data_size = DATA_CAPACITY;
    }

    for (int i = 0; i < data_size; i++) {
      data_angle[i] = _workAngle[i];
      data_distance[i] = _workDistance[i];
    }

    // Очищать хвост необязательно, потому что валидная зона задаётся data_size.
    // Но для отладки можно обнулить:
    for (int i = data_size; i < DATA_CAPACITY; i++) {
      data_angle[i] = 0.0f;
      data_distance[i] = 0;
    }

    unlockData();
  }

  void initPublicData() {
    data_size = 0;

    for (int i = 0; i < DATA_CAPACITY; i++) {
      data_angle[i] = 0.0f;
      data_distance[i] = 0;
    }
  }

  void initWorkData() {
    _workSize = 0;
    _hasLastWorkAngle = false;
    _lastWorkAngle = 0.0f;

    for (int i = 0; i < DATA_CAPACITY; i++) {
      _workAngle[i] = 0.0f;
      _workDistance[i] = 0;
    }
  }

  static uint16_t readU16LE(const uint8_t *p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
  }

  static uint8_t calcCRC8(const uint8_t *data, uint8_t len) {
    constexpr uint8_t polynomial = 0x4D;
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++) {
      crc ^= data[i];

      for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80) {
          crc = static_cast<uint8_t>((crc << 1) ^ polynomial);
        } else {
          crc = static_cast<uint8_t>(crc << 1);
        }
      }
    }

    return crc;
  }

  void lockData() {
    if (_dataMutex != nullptr) {
      xSemaphoreTake(_dataMutex, portMAX_DELAY);
    }
  }

  void unlockData() {
    if (_dataMutex != nullptr) {
      xSemaphoreGive(_dataMutex);
    }
  }
};

#endif