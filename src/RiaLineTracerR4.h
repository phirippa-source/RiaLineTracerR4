#pragma once
#include <Arduino.h>

class RiaLineSensorR4 {
public:
  static constexpr uint8_t MAX_SENSORS = 8;

  RiaLineSensorR4(const uint8_t *pins, uint8_t numSensors, int8_t emitterPin = -1);

  // emitterAlwaysOn=true면 begin()에서 계속 ON
  // false면 readRaw()가 읽을 때만 ON (전류 절감)
  void begin(bool emitterAlwaysOn = true);

  // RC raw read: time-to-discharge in microseconds, clipped to timeout
  // return: used_us (early-exit may be < timeout)
  uint16_t readRaw(uint16_t *values,
                   uint16_t timeoutMicros = 2500,
                   uint8_t chargeMicros = 10);

  void resetCalibration();
  void calibrate(uint16_t iterations = 200,
                 uint16_t timeoutMicros = 2500,
                 uint8_t chargeMicros = 10,
                 uint16_t interDelayMs = 5);

  // normalized 0..1000 (white..black) by default
  void readCalibrated(uint16_t *values,
                      uint16_t timeoutMicros = 2500,
                      uint8_t chargeMicros = 10);

  // 기존 스타일(위치만)
  int32_t readLine(bool whiteLine = false,
                   uint16_t timeoutMicros = 2500,
                   uint8_t chargeMicros = 10,
                   uint16_t noiseThreshold = 50);

  // lineLost까지 외부로 알려주는 버전(안정성에 핵심)
  int32_t readLineWithStatus(bool &lineLost,
                             bool whiteLine = false,
                             uint16_t timeoutMicros = 2500,
                             uint8_t chargeMicros = 10,
                             uint16_t noiseThreshold = 50);

  uint8_t numSensors() const { return _numSensors; }
  int8_t emitterPin() const { return _emitterPin; }
  bool calibrated() const { return _calibrated; }
  int32_t lastPosition() const { return _lastPosition; }

private:
  uint8_t _numSensors;
  int8_t  _emitterPin;
  bool    _calibrated;
  bool    _emitterAlwaysOn;
  int32_t _lastPosition;

  uint8_t  _pins[MAX_SENSORS];
  uint16_t _calibMin[MAX_SENSORS];
  uint16_t _calibMax[MAX_SENSORS];

  void setEmitter(bool on);
};

class RiaLinePID {
public:
  RiaLinePID();

  void setGains(float kp, float ki, float kd);
  void setIntegralLimit(float limitAbs);
  void reset();

  float update(float error, float dtSeconds);

private:
  float _kp, _ki, _kd;
  float _integral;
  float _prevError;
  float _integralLimitAbs;
  bool  _hasPrev;
};

struct RiaMotorCommand {
  int16_t left;
  int16_t right;
  int32_t position;  // 0..(N-1)*1000
  int16_t error;     // position - center (clamped)
  bool    lineLost;
};

class RiaLineTracerR4 {
public:
  RiaLineTracerR4(const uint8_t *sensorPins,
                  uint8_t numSensors,
                  int8_t emitterPin = -1);

  void begin(bool emitterAlwaysOn = true);

  void calibrate(uint16_t iterations = 200,
                 uint16_t timeoutMicros = 2500,
                 uint8_t chargeMicros = 10,
                 uint16_t interDelayMs = 5);

  // lineLost 대응 정책
  // - holdLastOnLost: 라인을 놓치면 마지막 correction 유지(기본 true)
  // - searchOnLost: 라인을 놓치면 약하게 한쪽으로 회전하며 탐색(holdLast보다 적극적)
  void setHoldLastOnLost(bool enable) { _holdLastOnLost = enable; }
  void setSearchOnLost(bool enable) { _searchOnLost = enable; }
  void setSearchTurn(int16_t turn) { _searchTurn = turn; } // 탐색 회전 세기(양수면 좌회전 기준)

  // One control step
  RiaMotorCommand step(int16_t baseSpeed,
                       int16_t maxSpeed,
                       bool whiteLine = false,
                       uint16_t timeoutMicros = 2500,
                       uint8_t chargeMicros = 10,
                       uint16_t noiseThreshold = 50);

  RiaLineSensorR4& sensor() { return _sensor; }
  RiaLinePID& pid() { return _pid; }

private:
  RiaLineSensorR4 _sensor;
  RiaLinePID _pid;

  uint32_t _lastStepMicros;
  float    _lastCorrection;

  bool _holdLastOnLost;
  bool _searchOnLost;
  int16_t _searchTurn;
};
