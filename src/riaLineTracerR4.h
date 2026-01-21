#include "riaLineTracerR4.h"

// -------------------- Utility --------------------
int16_t RiaLineTracerR4::clampI16(int32_t v, int16_t lo, int16_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

// -------------------- Constructor --------------------
RiaLineTracerR4::RiaLineTracerR4(const uint8_t *sensorPins, uint8_t numSensors, int8_t emitterPin)
: _numSensors(numSensors),
  _emitterPin(emitterPin),
  _emitterAlwaysOn(true),
  _calibrated(false),
  _lastPosition(0),
  // PID defaults (tune later)
  _kp(0.18f), _ki(0.0f), _kd(1.2f),
  _integral(0.0f), _prevError(0.0f), _hasPrev(false),
  _integralLimitAbs(1500.0f),
  // timing
  _lastStepMicros(0),
  _lastCorrection(0.0f),
  // lineLost strategy
  _holdLastOnLost(true),
  _searchOnLost(false),
  _searchTurn(90),
  // default Zumo Shield pins
  _pinDirL(8), _pinPwmL(10),
  _pinDirR(7), _pinPwmR(9),
  _dirLowIsForward(true),
  _minPwmWhenMoving(0)
{
  if (_numSensors > MAX_SENSORS) _numSensors = MAX_SENSORS;

  for (uint8_t i = 0; i < _numSensors; i++) {
    _pins[i] = sensorPins[i];
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }

  if (_numSensors > 0) {
    _lastPosition = ((int32_t)(_numSensors - 1) * 1000L) / 2;
  }
}

// -------------------- begin --------------------
void RiaLineTracerR4::begin(bool emitterAlwaysOn)
{
  _emitterAlwaysOn = emitterAlwaysOn;

  if (_emitterPin >= 0) {
    pinMode(_emitterPin, OUTPUT);
    setEmitter(_emitterAlwaysOn);
  }

  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  initMotorPins();

  resetPID();
  _lastStepMicros = micros();
  _lastCorrection = 0.0f;
}

// -------------------- Emitter --------------------
void RiaLineTracerR4::setEmitter(bool on)
{
  if (_emitterPin < 0) return;
  digitalWrite(_emitterPin, on ? HIGH : LOW);
}

// -------------------- Calibration --------------------
void RiaLineTracerR4::resetCalibration()
{
  _calibrated = false;
  for (uint8_t i = 0; i < _numSensors; i++) {
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }
}

void RiaLineTracerR4::calibrate(uint16_t iterations,
                                uint16_t timeoutMicros,
                                uint8_t chargeMicros,
                                uint16_t interDelayMs)
{
  uint16_t raw[MAX_SENSORS];

  for (uint16_t n = 0; n < iterations; n++) {
    readRawInternal(raw, timeoutMicros, chargeMicros);

    for (uint8_t i = 0; i < _numSensors; i++) {
      uint16_t v = raw[i];
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

// -------------------- Sensor reads --------------------
uint16_t RiaLineTracerR4::readRaw(uint16_t *rawValues, uint16_t timeoutMicros, uint8_t chargeMicros)
{
  return readRawInternal(rawValues, timeoutMicros, chargeMicros);
}

uint16_t RiaLineTracerR4::readRawInternal(uint16_t *values, uint16_t timeoutMicros, uint8_t chargeMicros)
{
  if (timeoutMicros > 32767) timeoutMicros = 32767;

  if (_emitterPin >= 0 && !_emitterAlwaysOn) {
    setEmitter(true);
    if (chargeMicros) delayMicroseconds(chargeMicros);
  }

  // 1) charge
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], HIGH);
  }
  if (chargeMicros) delayMicroseconds(chargeMicros);

  // 2) init
  for (uint8_t i = 0; i < _numSensors; i++) {
    values[i] = timeoutMicros;
  }

  // 3) release
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  // 4) measure (early-exit + pending mask)
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

  if (_emitterPin >= 0 && !_emitterAlwaysOn) {
    setEmitter(false);
  }

  return elapsed;
}

void RiaLineTracerR4::readCalibrated(uint16_t *calibratedValues,
                                     uint16_t timeoutMicros,
                                     uint8_t chargeMicros)
{
  uint16_t raw[MAX_SENSORS];
  readRawInternal(raw, timeoutMicros, chargeMicros);

  for (uint8_t i = 0; i < _numSensors; i++) {
    const uint16_t v = raw[i];
    const uint16_t minv = _calibMin[i];
    const uint16_t maxv = _calibMax[i];

    if (!_calibrated || maxv <= minv) {
      calibratedValues[i] = 0;
    } else {
      int32_t num = (int32_t)v - (int32_t)minv;
      if (num < 0) num = 0;

      uint16_t norm = (uint16_t)((num * 1000L) / (maxv - minv));
      if (norm > 1000) norm = 1000;
      calibratedValues[i] = norm;
    }
  }
}

int32_t RiaLineTracerR4::readLine(bool whiteLine,
                                 uint16_t timeoutMicros,
                                 uint8_t chargeMicros,
                                 uint16_t noiseThreshold)
{
  bool dummyLost = false;
  return readLineWithStatus(dummyLost, whiteLine, timeoutMicros, chargeMicros, noiseThreshold);
}

int32_t RiaLineTracerR4::readLineWithStatus(bool &lineLost,
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

// -------------------- PID --------------------
void RiaLineTracerR4::setPID(float kp, float ki, float kd)
{
  _kp = kp; _ki = ki; _kd = kd;
}

void RiaLineTracerR4::setIntegralLimit(float limitAbs)
{
  if (limitAbs < 0) limitAbs = -limitAbs;
  _integralLimitAbs = limitAbs;
}

void RiaLineTracerR4::resetPID()
{
  _integral = 0.0f;
  _prevError = 0.0f;
  _hasPrev = false;
}

float RiaLineTracerR4::pidUpdate(float error, float dtSeconds)
{
  if (dtSeconds <= 0.0f) dtSeconds = 0.001f;

  _integral += error * dtSeconds;
  if (_integral > _integralLimitAbs) _integ_
