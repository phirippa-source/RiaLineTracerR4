#include "riaLineTracerR4.h"

// =========================
// RiaLineSensorR4
// =========================
RiaLineSensorR4::RiaLineSensorR4(const uint8_t *pins, uint8_t numSensors, int8_t emitterPin)
: _numSensors(numSensors),
  _emitterPin(emitterPin),
  _calibrated(false),
  _emitterAlwaysOn(true),
  _lastPosition(0)
{
  if (_numSensors > MAX_SENSORS) _numSensors = MAX_SENSORS;

  for (uint8_t i = 0; i < _numSensors; i++) {
    _pins[i] = pins[i];
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }

  if (_numSensors > 0) {
    _lastPosition = ((int32_t)(_numSensors - 1) * 1000L) / 2;
  }
}

void RiaLineSensorR4::setEmitter(bool on)
{
  if (_emitterPin < 0) return;
  digitalWrite(_emitterPin, on ? HIGH : LOW);
}

void RiaLineSensorR4::begin(bool emitterAlwaysOn)
{
  _emitterAlwaysOn = emitterAlwaysOn;

  if (_emitterPin >= 0) {
    pinMode(_emitterPin, OUTPUT);
    setEmitter(_emitterAlwaysOn);
  }

  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }
}

uint16_t RiaLineSensorR4::readRaw(uint16_t *values, uint16_t timeoutMicros, uint8_t chargeMicros)
{
  if (timeoutMicros > 32767) timeoutMicros = 32767;

  // emitter control
  if (_emitterPin >= 0 && !_emitterAlwaysOn) {
    setEmitter(true);
    if (chargeMicros) delayMicroseconds(chargeMicros);
  }

  // 1) charge all sensors
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], HIGH);
  }
  if (chargeMicros) delayMicroseconds(chargeMicros);

  // 2) init with timeout
  for (uint8_t i = 0; i < _numSensors; i++) {
    values[i] = timeoutMicros;
  }

  // 3) release to input
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  // 4) discharge timing with early exit + pending mask
  const uint32_t start = micros();

  uint8_t pendingMask = 0;
  for (uint8_t i = 0; i < _numSensors; i++) pendingMask |= (1u << i);

  uint16_t elapsed = 0;

  while (pendingMask != 0) {
    elapsed = (uint16_t)(micros() - start);
    if (elapsed >= timeoutMicros) {
      elapsed = timeoutMicros;
      break;
    }

    for (uint8_t i = 0; i < _numSensors; i++) {
      const uint8_t bit = (1u << i);
      if ((pendingMask & bit) == 0) continue;

      if (digitalRead(_pins[i]) == LOW) {
        values[i] = elapsed;
        pendingMask &= (uint8_t)~bit;
        if (pendingMask == 0) break;
      }
    }
  }

  // emitter off (if not always on)
  if (_emitterPin >= 0 && !_emitterAlwaysOn) {
    setEmitter(false);
  }

  return elapsed;
}

void RiaLineSensorR4::resetCalibration()
{
  _calibrated = false;
  for (uint8_t i = 0; i < _numSensors; i++) {
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }
}

void RiaLineSensorR4::calibrate(uint16_t iterations, uint16_t timeoutMicros, uint8_t chargeMicros, uint16_t interDelayMs)
{
  uint16_t raw[MAX_SENSORS];

  for (uint16_t n = 0; n < iterations; n++) {
    readRaw(raw, timeoutMicros, chargeMicros);

    for (uint8_t i = 0; i < _numSensors; i++) {
      const uint16_t v = raw[i];
      if (!_calibrated) {
        _calibMin[i] = v;
        _calibMax[i] = v;
      } else {
        if (v < _calibMin[i]) _calibMin[i] = v;
        if (v > _calibMax[i]) _calibMax[i] = v;
      }
    }
    _calibrated = true;
    if (interDelayMs) delay(interDelayMs);
  }
}

void RiaLineSensorR4::readCalibrated(uint16_t *values, uint16_t timeoutMicros, uint8_t chargeMicros)
{
  uint16_t raw[MAX_SENSORS];
  readRaw(raw, timeoutMicros, chargeMicros);

  for (uint8_t i = 0; i < _numSensors; i++) {
    const uint16_t v    = raw[i];
    const uint16_t minv = _calibMin[i];
    const uint16_t maxv = _calibMax[i];

    if (!_calibrated || maxv <= minv) {
      values[i] = 0;
    } else {
      int32_t num = (int32_t)v - (int32_t)minv;
      if (num < 0) num = 0;

      uint16_t norm = (uint16_t)((num * 1000L) / (maxv - minv));
      if (norm > 1000) norm = 1000;
      values[i] = norm;
    }
  }
}

int32_t RiaLineSensorR4::readLine(bool whiteLine, uint16_t timeoutMicros, uint8_t chargeMicros, uint16_t noiseThreshold)
{
  bool dummyLost = false;
  return readLineWithStatus(dummyLost, whiteLine, timeoutMicros, chargeMicros, noiseThreshold);
}

int32_t RiaLineSensorR4::readLineWithStatus(bool &lineLost,
                                            bool whiteLine,
                                            uint16_t timeoutMicros,
                                            uint8_t chargeMicros,
                                            uint16_t noiseThreshold)
{
  uint16_t cal[MAX_SENSORS];
  readCalibrated(cal, timeoutMicros, chargeMicros);

  uint32_t weightedSum = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < _numSensors; i++) {
    uint16_t v = cal[i];
    if (whiteLine) v = 1000 - v;
    if (v < noiseThreshold) continue;

    weightedSum += (uint32_t)v * (uint32_t)(i * 1000UL);
    sum += v;
  }

  if (sum == 0) {
    lineLost = true;
    return _lastPosition;
  }

  lineLost = false;
  _lastPosition = (int32_t)(weightedSum / sum);
  return _lastPosition;
}

// =========================
// RiaLinePID
// =========================
RiaLinePID::RiaLinePID()
: _kp(0.20f), _ki(0.0f), _kd(1.50f),
  _integral(0.0f), _prevError(0.0f),
  _integralLimitAbs(2000.0f),
  _hasPrev(false)
{}

void RiaLinePID::setGains(float kp, float ki, float kd)
{
  _kp = kp; _ki = ki; _kd = kd;
}

void RiaLinePID::setIntegralLimit(float limitAbs)
{
  if (limitAbs < 0) limitAbs = -limitAbs;
  _integralLimitAbs = limitAbs;
}

void RiaLinePID::reset()
{
  _integral = 0.0f;
  _prevError = 0.0f;
  _hasPrev = false;
}

float RiaLinePID::update(float error, float dtSeconds)
{
  if (dtSeconds <= 0.0f) dtSeconds = 0.001f;

  // Integral (anti-windup)
  _integral += error * dtSeconds;
  if (_integral > _integralLimitAbs) _integral = _integralLimitAbs;
  if (_integral < -_integralLimitAbs) _integral = -_integralLimitAbs;

  float derivative = 0.0f;
  if (_hasPrev) {
    derivative = (error - _prevError) / dtSeconds;
  } else {
    _hasPrev = true;
  }
  _prevError = error;

  return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

// =========================
// RiaLineTracerR4
// =========================
RiaLineTracerR4::RiaLineTracerR4(const uint8_t *sensorPins, uint8_t numSensors, int8_t emitterPin)
: _sensor(sensorPins, numSensors, emitterPin),
  _pid(),
  _lastStepMicros(0),
  _lastCorrection(0.0f),
  _holdLastOnLost(true),
  _searchOnLost(false),
  _searchTurn(80)
{}

void RiaLineTracerR4::begin(bool emitterAlwaysOn)
{
  _sensor.begin(emitterAlwaysOn);
  _pid.reset();
  _lastStepMicros = micros();
  _lastCorrection = 0.0f;
}

void RiaLineTracerR4::calibrate(uint16_t iterations, uint16_t timeoutMicros, uint8_t chargeMicros, uint16_t interDelayMs)
{
  _sensor.calibrate(iterations, timeoutMicros, chargeMicros, interDelayMs);
}

RiaMotorCommand RiaLineTracerR4::step(int16_t baseSpeed,
                                      int16_t maxSpeed,
                                      bool whiteLine,
                                      uint16_t timeoutMicros,
                                      uint8_t chargeMicros,
                                      uint16_t noiseThreshold)
{
  const uint32_t now = micros();
  float dt = (now - _lastStepMicros) / 1000000.0f;
  _lastStepMicros = now;

  bool lineLost = false;
  const int32_t pos = _sensor.readLineWithStatus(lineLost, whiteLine, timeoutMicros, chargeMicros, noiseThreshold);
  const int32_t center = ((int32_t)(_sensor.numSensors() - 1) * 1000L) / 2;

  int32_t err32 = pos - center;

  float correction = _pid.update((float)err32, dt);

  // lineLost handling: 안정성 우선
  if (lineLost) {
    if (_searchOnLost) {
      // 적극 탐색: 한쪽으로 약하게 회전하며 라인 재획득 시도
      correction = (float)_searchTurn;
    } else if (_holdLastOnLost) {
      // 소극 안정: 마지막 correction 유지
      correction = _lastCorrection;
    } else {
      // 직진으로 복귀
      correction = 0.0f;
    }
  }

  _lastCorrection = correction;

  // motor mixing
  int32_t left  = (int32_t)baseSpeed + (int32_t)correction;
  int32_t right = (int32_t)baseSpeed - (int32_t)correction;

  // clamp
  if (left  >  maxSpeed) left  =  maxSpeed;
  if (left  < -maxSpeed) left  = -maxSpeed;
  if (right >  maxSpeed) right =  maxSpeed;
  if (right < -maxSpeed) right = -maxSpeed;

  // error clamp to int16
  int16_t err16;
  if (err32 >  32767) err16 = 32767;
  else if (err32 < -32768) err16 = -32768;
  else err16 = (int16_t)err32;

  RiaMotorCommand cmd;
  cmd.left = (int16_t)left;
  cmd.right = (int16_t)right;
  cmd.position = pos;
  cmd.error = err16;
  cmd.lineLost = lineLost;
  return cmd;
}
