#pragma once
#include <Arduino.h>

/*
  RiaLineTracerR4 (Educational version)

  Goals:
  - Students can read the control pipeline in .ino easily:
      setup(): begin() + calibrate...
      loop():  readLine() -> computeError() -> computeCorrection() -> drive()
  - RC sensor reading, calibration, PID, motor PWM are separated as clean functions.

  Default assumptions:
  - UNO R4 WiFi
  - Zumo Reflectance Sensor Array (RC timing)
  - Zumo Shield style motor control pins (default):
      leftDir=8 leftPwm=10 rightDir=7 rightPwm=9
  - emitterPin default = 2
*/

class RiaLineTracerR4 {
public:
  static constexpr uint8_t MAX_SENSORS = 8;

  // Result of line reading
  struct LineResult {
    bool    lost;       // true if line is not found
    int32_t position;   // 0..(N-1)*1000, left=0, right=(N-1)*1000
  };

  // Motor command (signed PWM)
  struct MotorCommand {
    int16_t left;       // -255..255 (clamped by maxSpeed)
    int16_t right;      // -255..255 (clamped by maxSpeed)
  };

public:
  // Read calibrated values AND line position in one sensor read
  // - calValues: output 0..1000 per sensor (size >= sensorCount())
  // - whiteLine: false=black line on white, true=white line on black
  // - noiseThreshold: ignore small values as noise
  LineResult readLineWithCal(uint16_t* calValues,
                             bool whiteLine = false,
                             uint16_t timeoutUs = 2000,
                             uint8_t  chargeUs  = 10,
                             uint16_t noiseThreshold = 140);

  // Sensor info helpers (for examples / education)
  uint8_t sensorCount() const;
  int32_t centerPosition() const; // 0..(N-1)*1000 의 중앙값

  // pins: sensor pins in LEFT -> RIGHT order
  // numSensors: number of sensors (<= MAX_SENSORS)
  // emitterPin: IR emitter control pin (default = 2)
  explicit RiaLineTracerR4(const uint8_t* pins, uint8_t numSensors, int8_t emitterPin = 2);

  // Initialize sensor pins, motor pins, emitter control
  // emitterAlwaysOn=true keeps emitter on after begin() (stable for most cases)
  void begin(bool emitterAlwaysOn = true);

  // -----------------------
  // 1) Calibration (setup)
  // -----------------------
  void resetCalibration();

  // Manual calibration: user moves the robot over line and background
  void calibrateManual(uint16_t iterations = 200,
                       uint16_t timeoutUs = 2000,
                       uint8_t  chargeUs  = 10,
                       uint16_t interDelayMs = 5);

  // Auto calibration: spin left/right while accumulating min/max
  void calibrateSpin(uint16_t loops = 400,
                     int16_t  turnPwm = 130,
                     uint16_t block = 80,
                     uint16_t timeoutUs = 2000,
                     uint8_t  chargeUs  = 10,
                     uint8_t  perLoopDelayMs = 10);

  // Safer auto calibration: spin with guard and auto-finish
  // 목표: 라인 밖으로 크게 벗어나지 않으면서 모든 센서가 line+bg를 충분히 경험
  bool calibrateSpinSafe(uint16_t maxLoops = 600,
                         int16_t  turnPwm = 110,
                         uint16_t block = 60,
                         uint16_t timeoutUs = 2000,
                         uint8_t  chargeUs  = 10,
                         uint16_t noiseThreshold = 140,
                         uint16_t targetRange = 300,
                         uint8_t  stableNeed = 20,
                         uint8_t  perLoopDelayMs = 10);

  // -----------------------
  // 2) Read sensors (loop)
  // -----------------------
  // RC discharge timing raw(us)
  void readRaw(uint16_t* rawValues,
               uint16_t timeoutUs = 2000,
               uint8_t  chargeUs  = 10);

  // Calibrated values 0..1000 (0=white, 1000=black)
  void readCalibrated(uint16_t* calValues,
                      uint16_t timeoutUs = 2000,
                      uint8_t  chargeUs  = 10);

  // Read line position (weighted average)
  LineResult readLine(bool whiteLine = false,
                      uint16_t timeoutUs = 2000,
                      uint8_t  chargeUs  = 10,
                      uint16_t noiseThreshold = 140);

  // -----------------------
  // 3) Control (loop)
  // -----------------------
  // Convert position to error from center (negative=left, positive=right)
  int32_t computeError(int32_t position) const;

  // PID correction (integer output)
  int32_t computeCorrection(int32_t error);

  // Drive motors using baseSpeed and correction (applies PWM internally)
  void drive(int16_t baseSpeed, int32_t correction, int16_t maxSpeed);

  // Same as drive(), but returns the applied motor command (for logging/telemetry)
  MotorCommand driveWithReport(int16_t baseSpeed, int32_t correction, int16_t maxSpeed);

  // Get last applied motor command
  MotorCommand getLastMotorCommand() const;

  // Stop immediately
  void stop();

  // PID parameters
  void setPID(float kp, float ki, float kd);
  void setIntegralLimit(float limitAbs);
  void resetPID();

  // -----------------------
  // 4) Motor config
  // -----------------------
  void setMotorPins(uint8_t leftDir, uint8_t leftPwm, uint8_t rightDir, uint8_t rightPwm);

  // If motor direction is reversed, flip polarity
  // lowIsForward=true: DIR LOW=forward, HIGH=reverse
  void setDirPolarity(bool lowIsForward);

  // Motor dead-zone compensation (0 disables)
  void setMinPwmWhenMoving(uint8_t minPwm);

  // Low-level motor apply (signed PWM)
  void applyMotor(int16_t left, int16_t right);

private:
  // emitter helpers
  void ensureEmitterOnForRead_();
  void emitterOn_();
  void emitterOff_();

  // clamp helpers
  int16_t clampMotor_(int32_t v, int16_t maxAbs) const;
  uint8_t clampPwmAbs_(int32_t v) const;
  int32_t roundToInt_(float x) const;

private:
  // sensor
  uint8_t _numSensors;
  uint8_t _pins[MAX_SENSORS];

  // emitter
  int8_t _emitterPin;
  bool   _emitterAlwaysOn;

  // calibration
  bool     _calibrated;
  uint16_t _calibMin[MAX_SENSORS];
  uint16_t _calibMax[MAX_SENSORS];

  // line
  int32_t  _lastPosition;

  // PID state
  float    _kp, _ki, _kd;
  float    _integral;
  float    _integralLimitAbs;
  float    _lastDerivInput;
  uint32_t _lastPidMicros;

  // motor pins (default Zumo shield style)
  uint8_t _leftDirPin;
  uint8_t _leftPwmPin;
  uint8_t _rightDirPin;
  uint8_t _rightPwmPin;

  bool    _dirLowIsForward;
  uint8_t _minPwmWhenMoving;

  // last motor command
  MotorCommand _lastMotor;
};
