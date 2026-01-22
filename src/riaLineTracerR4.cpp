#include "RiaLineTracerR4.h"

// -------------------- ctor / begin --------------------

RiaLineTracerR4::RiaLineTracerR4(const uint8_t* pins, uint8_t numSensors, int8_t emitterPin)
: _numSensors(numSensors),
  _emitterPin(emitterPin),
  _emitterAlwaysOn(true),
  _calibrated(false),
  _lastPosition(0),
  _kp(0.08f), _ki(0.0f), _kd(0.01f),
  _integral(0.0f),
  _integralLimitAbs(1500.0f),
  _lastDerivInput(0.0f),
  _lastPidMicros(0),
  _leftDirPin(8), _leftPwmPin(10), _rightDirPin(7), _rightPwmPin(9),
  _dirLowIsForward(true),
  _minPwmWhenMoving(0)
{
  if (_numSensors > MAX_SENSORS) _numSensors = MAX_SENSORS;

  for (uint8_t i = 0; i < _numSensors; i++) {
    _pins[i] = pins[i];
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }

  // 초기 lastPosition은 중앙
  if (_numSensors > 0) {
    _lastPosition = (int32_t)(_numSensors - 1) * 1000L / 2;
  }
}

void RiaLineTracerR4::begin(bool emitterAlwaysOn)
{
  _emitterAlwaysOn = emitterAlwaysOn;

  // emitter
  if (_emitterPin >= 0) {
    pinMode((uint8_t)_emitterPin, OUTPUT);
    digitalWrite((uint8_t)_emitterPin, _emitterAlwaysOn ? HIGH : LOW);
  }

  // sensor pins
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  // motor pins
  pinMode(_leftDirPin, OUTPUT);
  pinMode(_leftPwmPin, OUTPUT);
  pinMode(_rightDirPin, OUTPUT);
  pinMode(_rightPwmPin, OUTPUT);

  stop();
  resetPID();
}

// -------------------- emitter helpers --------------------

void RiaLineTracerR4::emitterOn_()
{
  if (_emitterPin >= 0) digitalWrite((uint8_t)_emitterPin, HIGH);
}

void RiaLineTracerR4::emitterOff_()
{
  if (_emitterPin >= 0) digitalWrite((uint8_t)_emitterPin, LOW);
}

void RiaLineTracerR4::ensureEmitterOnForRead_()
{
  if (_emitterPin < 0) return;
  if (_emitterAlwaysOn) return;
  emitterOn_();
  delayMicroseconds(10);
}

// -------------------- calibration --------------------

void RiaLineTracerR4::resetCalibration()
{
  _calibrated = false;
  for (uint8_t i = 0; i < _numSensors; i++) {
    _calibMin[i] = 0xFFFF;
    _calibMax[i] = 0;
  }
}

void RiaLineTracerR4::calibrateManual(uint16_t iterations,
                                      uint16_t timeoutUs,
                                      uint8_t  chargeUs,
                                      uint16_t interDelayMs)
{
  uint16_t raw[MAX_SENSORS];

  for (uint16_t n = 0; n < iterations; n++) {
    readRaw(raw, timeoutUs, chargeUs);

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
    if (interDelayMs > 0) delay(interDelayMs);
  }
}

void RiaLineTracerR4::calibrateSpin(uint16_t loops,
                                    int16_t  turnPwm,
                                    uint16_t block,
                                    uint16_t timeoutUs,
                                    uint8_t  chargeUs,
                                    uint8_t  perLoopDelayMs)
{
  if (loops == 0) return;
  if (block == 0) block = 1;

  resetCalibration();

  for (uint16_t i = 0; i < loops; i++) {
    // block마다 방향 전환
    const bool dir = ((i / block) % 2 == 0);

    if (dir) applyMotor(+turnPwm, -turnPwm);
    else     applyMotor(-turnPwm, +turnPwm);

    // 1회씩 누적
    calibrateManual(1, timeoutUs, chargeUs, 0);

    if (perLoopDelayMs > 0) delay(perLoopDelayMs);
  }

  stop();
}

// -------------------- sensor read (RC) --------------------

void RiaLineTracerR4::readRaw(uint16_t* rawValues, uint16_t timeoutUs, uint8_t chargeUs)
{
  ensureEmitterOnForRead_();

  // 1) 충전: OUTPUT HIGH
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], HIGH);
  }
  if (chargeUs > 0) delayMicroseconds(chargeUs);

  // 기본값은 timeout
  for (uint8_t i = 0; i < _numSensors; i++) rawValues[i] = timeoutUs;

  // 2) INPUT으로 바꾸고 LOW로 떨어지는 시간 측정
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  uint16_t pendingMask = 0;
  for (uint8_t i = 0; i < _numSensors; i++) pendingMask |= (uint16_t)(1u << i);

  const uint32_t start = micros();

  while (pendingMask != 0) {
    const uint32_t elapsed = micros() - start;
    if (elapsed >= timeoutUs) break;

    for (uint8_t i = 0; i < _numSensors; i++) {
      const uint16_t bit = (uint16_t)(1u << i);
      if ((pendingMask & bit) == 0) continue;

      if (digitalRead(_pins[i]) == LOW) {
        rawValues[i] = (uint16_t)elapsed;
        pendingMask &= (uint16_t)~bit;
        if (pendingMask == 0) break;
      }
    }
  }

  if (_emitterPin >= 0 && !_emitterAlwaysOn) emitterOff_();
}

void RiaLineTracerR4::readCalibrated(uint16_t* calValues, uint16_t timeoutUs, uint8_t chargeUs)
{
  uint16_t raw[MAX_SENSORS];
  readRaw(raw, timeoutUs, chargeUs);

  for (uint8_t i = 0; i < _numSensors; i++) {
    const uint16_t v = raw[i];
    const uint16_t minv = _calibMin[i];
    const uint16_t maxv = _calibMax[i];

    if (!_calibrated || maxv <= minv) {
      calValues[i] = 0;
      continue;
    }

    int32_t num = (int32_t)v - (int32_t)minv;
    if (num < 0) num = 0;

    const uint32_t den = (uint32_t)(maxv - minv);
    uint32_t norm = (uint32_t)num * 1000UL / den;
    if (norm > 1000UL) norm = 1000UL;

    calValues[i] = (uint16_t)norm; // 0..1000
  }
}

RiaLineTracerR4::LineResult RiaLineTracerR4::readLine(bool whiteLine,
                                                      uint16_t timeoutUs,
                                                      uint8_t  chargeUs,
                                                      uint16_t noiseThreshold)
{
  uint16_t cal[MAX_SENSORS];
  readCalibrated(cal, timeoutUs, chargeUs);

  uint32_t weightedSum = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < _numSensors; i++) {
    uint16_t v = cal[i];

    // whiteLine 모드면 뒤집기
    if (whiteLine) v = 1000 - v;

    // 너무 작은 값은 노이즈로 무시
    if (v < noiseThreshold) continue;

    weightedSum += (uint32_t)v * (uint32_t)(i * 1000UL);
    sum += v;
  }

  LineResult out{};
  if (sum == 0) {
    out.lost = true;
    out.position = _lastPosition; // 마지막 위치 유지
    return out;
  }

  out.lost = false;
  out.position = (int32_t)(weightedSum / sum);
  _lastPosition = out.position;
  return out;
}

// -------------------- control (PID + drive) --------------------

int32_t RiaLineTracerR4::computeError(int32_t position) const
{
  const int32_t center = (int32_t)(_numSensors - 1) * 1000L / 2;
  return position - center;
}

int32_t RiaLineTracerR4::computeCorrection(int32_t error)
{
  // dt
  const uint32_t now = micros();
  float dt = (now - _lastPidMicros) * 1e-6f;
  _lastPidMicros = now;
  if (dt < 0.001f) dt = 0.001f;

  const float e = (float)error;

  // integral (clamp)
  _integral += e * dt;
  if (_integral >  _integralLimitAbs) _integral =  _integralLimitAbs;
  if (_integral < -_integralLimitAbs) _integral = -_integralLimitAbs;

  float deriv = 0.0f;
  if (_kd != 0.0f) {
    deriv = (e - _lastDerivInput) / dt;
    _lastDerivInput = e;
  }

  const float correction = (_kp * e) + (_ki * _integral) + (_kd * deriv);
  return roundToInt_(correction);
}

void RiaLineTracerR4::drive(int16_t baseSpeed, int32_t correction, int16_t maxSpeed)
{
  const int16_t maxAbs = (maxSpeed < 0) ? (int16_t)(-maxSpeed) : maxSpeed;

  const int32_t leftCmd  = (int32_t)baseSpeed + correction;
  const int32_t rightCmd = (int32_t)baseSpeed - correction;

  const int16_t L = clampMotor_(leftCmd,  maxAbs);
  const int16_t R = clampMotor_(rightCmd, maxAbs);

  applyMotor(L, R);
}

void RiaLineTracerR4::stop()
{
  applyMotor(0, 0);
}

// -------------------- PID config --------------------

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
  _lastDerivInput = 0.0f;
  _lastPidMicros = micros();
}

// -------------------- motor helpers --------------------

uint8_t RiaLineTracerR4::clampPwmAbs_(int32_t v) const
{
  if (v < 0) v = -v;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

int16_t RiaLineTracerR4::clampMotor_(int32_t v, int16_t maxAbs) const
{
  if (v >  maxAbs) v =  maxAbs;
  if (v < -maxAbs) v = -maxAbs;
  return (int16_t)v;
}

int32_t RiaLineTracerR4::roundToInt_(float x) const
{
  return (x >= 0.0f) ? (int32_t)(x + 0.5f) : (int32_t)(x - 0.5f);
}

// -------------------- motor config / apply --------------------

void RiaLineTracerR4::setMotorPins(uint8_t leftDir, uint8_t leftPwm, uint8_t rightDir, uint8_t rightPwm)
{
  _leftDirPin = leftDir;
  _leftPwmPin = leftPwm;
  _rightDirPin = rightDir;
  _rightPwmPin = rightPwm;

  pinMode(_leftDirPin, OUTPUT);
  pinMode(_leftPwmPin, OUTPUT);
  pinMode(_rightDirPin, OUTPUT);
  pinMode(_rightPwmPin, OUTPUT);

  stop();
}

void RiaLineTracerR4::setDirPolarity(bool lowIsForward)
{
  _dirLowIsForward = lowIsForward;
}

void RiaLineTracerR4::setMinPwmWhenMoving(uint8_t minPwm)
{
  _minPwmWhenMoving = minPwm;
}

void RiaLineTracerR4::applyMotor(int16_t left, int16_t right)
{
  // Left
  const bool leftForward = (left >= 0);
  uint8_t leftPwm = clampPwmAbs_(left);
  if (leftPwm > 0 && _minPwmWhenMoving > 0 && leftPwm < _minPwmWhenMoving) {
    leftPwm = _minPwmWhenMoving;
  }

  uint8_t leftDirLevel;
  if (_dirLowIsForward) leftDirLevel = leftForward ? LOW : HIGH;
  else                 leftDirLevel = leftForward ? HIGH : LOW;

  digitalWrite(_leftDirPin, leftDirLevel);
  analogWrite(_leftPwmPin, leftPwm);

  // Right
  const bool rightForward = (right >= 0);
  uint8_t rightPwm = clampPwmAbs_(right);
  if (rightPwm > 0 && _minPwmWhenMoving > 0 && rightPwm < _minPwmWhenMoving) {
    rightPwm = _minPwmWhenMoving;
  }

  uint8_t rightDirLevel;
  if (_dirLowIsForward) rightDirLevel = rightForward ? LOW : HIGH;
  else                 rightDirLevel = rightForward ? HIGH : LOW;

  digitalWrite(_rightDirPin, rightDirLevel);
  analogWrite(_rightPwmPin, rightPwm);
}
