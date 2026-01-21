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
  _kp(0.18f), _ki(0.0f), _kd(1.2f),
  _integral(0.0f), _prevError(0.0f), _hasPrev(false),
  _integralLimitAbs(1500.0f),
  _lastStepMicros(0),
  _lastCorrection(0.0f),
  _holdLastOnLost(true),
  _searchOnLost(false),
  _searchTurn(90),
  // Default Zumo Shield pins (common mapping)
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
  // Safety cap: avoids accidentally huge loop time
  if (timeoutMicros > 32767) timeoutMicros = 32767;

  // emitter control
  if (_emitterPin >= 0 && !_emitterAlwaysOn) {
    setEmitter(true);
    if (chargeMicros) delayMicroseconds(chargeMicros);
  }

  // 1) Charge all sensors
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], HIGH);
  }
  if (chargeMicros) delayMicroseconds(chargeMicros);

  // 2) Init with timeout
  for (uint8_t i = 0; i < _numSensors; i++) {
    values[i] = timeoutMicros;
  }

  // 3) Release to input
  for (uint8_t i = 0; i < _numSensors; i++) {
    pinMode(_pins[i], INPUT);
  }

  // 4) Discharge timing with early-exit + pending mask
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
      calibratedValues[i] = norm; // 0..1000 (white..black)
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
    return _lastPosition; // keep last
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

  // integral (anti-windup)
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

// -------------------- Line-lost strategy --------------------
void RiaLineTracerR4::setHoldLastOnLost(bool enable) { _holdLastOnLost = enable; }
void RiaLineTracerR4::setSearchOnLost(bool enable) { _searchOnLost = enable; }
void RiaLineTracerR4::setSearchTurn(int16_t turn) { _searchTurn = turn; }

// -------------------- Motor PWM --------------------
void RiaLineTracerR4::setMotorPins(uint8_t leftDirPin, uint8_t leftPwmPin,
                                   uint8_t rightDirPin, uint8_t rightPwmPin)
{
  _pinDirL = leftDirPin;
  _pinPwmL = leftPwmPin;
  _pinDirR = rightDirPin;
  _pinPwmR = rightPwmPin;
  initMotorPins();
}

void RiaLineTracerR4::setDirPolarity(bool lowIsForward)
{
  _dirLowIsForward = lowIsForward;
}

void RiaLineTracerR4::setMinPwmWhenMoving(uint8_t minPwm)
{
  _minPwmWhenMoving = minPwm;
}

void RiaLineTracerR4::initMotorPins()
{
  pinMode(_pinDirL, OUTPUT);
  pinMode(_pinDirR, OUTPUT);
  pinMode(_pinPwmL, OUTPUT);
  pinMode(_pinPwmR, OUTPUT);

  // default safe stop
  analogWrite(_pinPwmL, 0);
  analogWrite(_pinPwmR, 0);
}

uint8_t RiaLineTracerR4::clampPwmFromCommand(int16_t cmd) const
{
  int32_t a = cmd;
  if (a < 0) a = -a;

  if (a > 255) a = 255;

  if (a > 0 && _minPwmWhenMoving > 0 && a < _minPwmWhenMoving) {
    a = _minPwmWhenMoving;
  }
  return (uint8_t)a;
}

void RiaLineTracerR4::setOneMotor(uint8_t pinDir, uint8_t pinPwm, int16_t cmd)
{
  if (cmd == 0) {
    analogWrite(pinPwm, 0);
    return;
  }

  const bool reverse = (cmd < 0);

  // direction polarity
  // lowIsForward=true:
  //   forward => LOW, reverse => HIGH
  // lowIsForward=false:
  //   forward => HIGH, reverse => LOW
  const bool dirLevel = _dirLowIsForward ? reverse : !reverse;
  digitalWrite(pinDir, dirLevel ? HIGH : LOW);

  const uint8_t pwm = clampPwmFromCommand(cmd);
  analogWrite(pinPwm, pwm);
}

void RiaLineTracerR4::applyMotor(int16_t left, int16_t right)
{
  setOneMotor(_pinDirL, _pinPwmL, left);
  setOneMotor(_pinDirR, _pinPwmR, right);
}

// -------------------- step() --------------------
RiaLineTracerR4::MotorCommand RiaLineTracerR4::step(int16_t baseSpeed,
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
  const int32_t pos = readLineWithStatus(lineLost, whiteLine, timeoutMicros, chargeMicros, noiseThreshold);

  const int32_t center = ((int32_t)(_numSensors - 1) * 1000L) / 2;
  int32_t err32 = pos - center;

  float correction = pidUpdate((float)err32, dt);

  // lineLost handling
  if (lineLost) {
    if (_searchOnLost) {
      correction = (float)_searchTurn;
    } else if (_holdLastOnLost) {
      correction = _lastCorrection;
    } else {
      correction = 0.0f;
    }
  }
  _lastCorrection = correction;

  // motor mix
  int32_t left  = (int32_t)baseSpeed + (int32_t)correction;
  int32_t right = (int32_t)baseSpeed - (int32_t)correction;

  // clamp by maxSpeed then by PWM(255)
  if (maxSpeed < 0) maxSpeed = -maxSpeed;
  if (maxSpeed > 255) maxSpeed = 255;

  left  = (left  >  maxSpeed) ?  maxSpeed : left;
  left  = (left  < -maxSpeed) ? -maxSpeed : left;
  right = (right >  maxSpeed) ?  maxSpeed : right;
  right = (right < -maxSpeed) ? -maxSpeed : right;

  // Apply to hardware
  applyMotor((int16_t)left, (int16_t)right);

  // package result
  MotorCommand cmd;
  cmd.left = (int16_t)left;
  cmd.right = (int16_t)right;
  cmd.position = pos;
  cmd.error = clampI16(err32, (int16_t)-32768, (int16_t)32767);
  cmd.lineLost = lineLost;
  return cmd;
}
