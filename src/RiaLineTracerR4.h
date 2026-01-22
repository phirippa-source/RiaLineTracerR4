#pragma once
#include <Arduino.h>

/*
  RiaLineTracerR4 (Educational version)
  - 목적: 초보자에게 "라인 추종 흐름"이 setup/loop에서 그대로 보이도록 설계
  - 센서(RC) 읽기/보정, 라인 위치 계산, PID 보정값 계산, 모터 PWM 적용을 함수로 분리
*/

class RiaLineTracerR4 {
public:
  static constexpr uint8_t MAX_SENSORS = 8;

  // 라인 읽기 결과(교육용으로 "무엇을 얻었는지" 명확)
  struct LineResult {
    bool    lost;       // 라인을 못 찾았는지
    int32_t position;   // 0..(N-1)*1000 (왼쪽=0, 오른쪽=(N-1)*1000)
  };
  struct MotorCommand {
    int16_t left;
    int16_t right;
  };

  MotorCommand driveWithReport(int16_t baseSpeed, int32_t correction, int16_t maxSpeed);
  MotorCommand getLastMotorCommand() const;


public:
  // pins: 센서 핀(LEFT -> RIGHT 순서)
  // numSensors: 센서 개수(<=MAX_SENSORS)
  // emitterPin: IR emitter 제어 핀(기본값=2)
  explicit RiaLineTracerR4(const uint8_t* pins, uint8_t numSensors, int8_t emitterPin = 2);

  // emitterAlwaysOn=true 이면 begin() 이후 emitter를 계속 켜둠(일반적으로 안정적)
  void begin(bool emitterAlwaysOn = true);

  // -----------------------
  // 1) Calibration (setup에서 보이도록)
  // -----------------------
  void resetCalibration();

  // (수동) 사람이 AGV를 움직이면서 캘리브레이션할 때 사용
  void calibrateManual(uint16_t iterations = 200,
                       uint16_t timeoutUs = 2000,
                       uint8_t  chargeUs  = 10,
                       uint16_t interDelayMs = 5);

  // (자동) 제자리 좌/우 회전하면서 캘리브레이션(교육용: setup에서 호출)
  void calibrateSpin(uint16_t loops = 400,
                     int16_t  turnPwm = 130,
                     uint16_t block = 80,
                     uint16_t timeoutUs = 2000,
                     uint8_t  chargeUs  = 10,
                     uint8_t  perLoopDelayMs = 10);

  // -----------------------
  // 2) Read sensors (loop에서 보이도록)
  // -----------------------
  // RC 방전 시간(raw, us) 읽기
  void readRaw(uint16_t* rawValues,
               uint16_t timeoutUs = 2000,
               uint8_t  chargeUs  = 10);

  // 보정된 값(0..1000) 읽기: 0(흰색) ~ 1000(검정)
  void readCalibrated(uint16_t* calValues,
                      uint16_t timeoutUs = 2000,
                      uint8_t  chargeUs  = 10);

  // "라인 위치" 읽기(교육용: loop에서 첫 단계로 호출)
  LineResult readLine(bool whiteLine = false,
                      uint16_t timeoutUs = 2000,
                      uint8_t  chargeUs  = 10,
                      uint16_t noiseThreshold = 140);

  // -----------------------
  // 3) Control (loop에서 보이도록)
  // -----------------------
  // position을 center 기준 error로 변환 (음수=왼쪽, 양수=오른쪽)
  int32_t computeError(int32_t position) const;

  // PID로 correction 계산(교육용: loop에서 보이도록)
  int32_t computeCorrection(int32_t error);

  // baseSpeed + correction을 좌/우 모터로 혼합하고 PWM으로 적용
  void drive(int16_t baseSpeed, int32_t correction, int16_t maxSpeed);

  // 즉시 정지
  void stop();

  // PID 파라미터
  void setPID(float kp, float ki, float kd);
  void setIntegralLimit(float limitAbs);
  void resetPID();

  // -----------------------
  // 4) Motor config (기본은 Zumo Shield 핀)
  // -----------------------
  // 기본값(권장): leftDir=8 leftPwm=10 rightDir=7 rightPwm=9
  void setMotorPins(uint8_t leftDir, uint8_t leftPwm, uint8_t rightDir, uint8_t rightPwm);

  // 모터 방향이 반대면 false/true를 바꾸어 해결 가능
  // lowIsForward=true: DIR LOW=정방향, HIGH=역방향
  void setDirPolarity(bool lowIsForward);

  // 저속 데드존이 있으면 최소 PWM 강제 (예: 30~60)
  void setMinPwmWhenMoving(uint8_t minPwm);

  // 저수준: 좌/우 모터에 signed 명령(-255..255)을 바로 적용
  void applyMotor(int16_t left, int16_t right);

private:
  // 내부 helper
  void ensureEmitterOnForRead_();
  void emitterOn_();
  void emitterOff_();

  int16_t clampMotor_(int32_t v, int16_t maxAbs) const;
  uint8_t clampPwmAbs_(int32_t v) const;
  int32_t roundToInt_(float x) const;
  MotorCommand _lastMotor{0, 0};


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
  int32_t  _lastPosition; // lost일 때 마지막 위치 유지

  // PID state
  float    _kp, _ki, _kd;
  float    _integral;
  float    _integralLimitAbs;
  float    _lastDerivInput;
  uint32_t _lastPidMicros;

  // motor pins (Zumo Shield default)
  uint8_t _leftDirPin;
  uint8_t _leftPwmPin;
  uint8_t _rightDirPin;
  uint8_t _rightPwmPin;

  bool    _dirLowIsForward;
  uint8_t _minPwmWhenMoving;
};
