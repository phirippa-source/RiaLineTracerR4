#include <RiaLineTracerR4.h>

// Sensor pins (LEFT -> RIGHT)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

// 시작 전 대기(학생이 트랙 위에 올려놓을 시간)
static constexpr uint16_t START_DELAY_MS = 1500;

// PID (수업에서 보여야 하는 핵심 파라미터)
static constexpr float KP = 0.08f;
static constexpr float KI = 0.0f;
static constexpr float KD = 0.01f;

// 주행 속도 (수업에서 조절해볼 핵심 파라미터)
static constexpr int16_t BASE_SPEED = 170;
static constexpr int16_t MAX_SPEED  = 220;

// -----------------------------
// Stop pattern: "ALL BLACK" (optional)
// - 상수 선언을 최소화하기 위해, 임계값은 함수 내부에 둡니다.
// -----------------------------
bool isAllBlackStopPattern() {
  // 라이브러리 기본값으로 보정값 읽기:
  // readCalibrated(cal, timeoutUs=2000, chargeUs=10)
  uint16_t cal[RiaLineTracerR4::MAX_SENSORS];
  tracer.readCalibrated(cal);

  // 기본 readLine()과 일관되게 "검정=값 큼" 기준으로 판단
  // 임계값(ALL_BLACK_TH)과 연속횟수(ALL_BLACK_CONSECUTIVE)는
  // 수업 중 조절할 수 있도록 여기에서만 관리합니다.
  const uint16_t ALL_BLACK_TH = 900;
  const uint8_t  ALL_BLACK_CONSECUTIVE = 3;

  bool allBlack = true;
  for (int i = 0; i < 6; i++) {
    if (cal[i] < ALL_BLACK_TH) { allBlack = false; break; }
  }

  static uint8_t count = 0;
  if (allBlack) count++;
  else count = 0;

  return (count >= ALL_BLACK_CONSECUTIVE);
}

void setup() {
  Serial.begin(115200);

  // 1) 초기화
  tracer.begin(true);

  delay(START_DELAY_MS);

  // 2) 캘리브레이션: (기본값 사용)
  // calibrateSpin(loops=400, turnPwm=130, block=80, timeoutUs=2000, chargeUs=10, perLoopDelayMs=10)
  tracer.calibrateSpin();

  // 3) PID 설정
  tracer.setPID(KP, KI, KD);
  tracer.setIntegralLimit(1500.0f);
  tracer.resetPID();

  Serial.println("03_line_tracing_pid READY");
}

void loop() {
  // (옵션) 특정 패턴(6개 모두 검정)에서 정지
  // 필요 없으면 이 블록을 통째로 주석 처리하세요.
  if (isAllBlackStopPattern()) {
    tracer.stop();
    while (true) delay(100);
  }

  // (1) 라인 읽기: (기본값 사용)
  // readLine(whiteLine=false, timeoutUs=2000, chargeUs=10, noiseThreshold=140)
  auto line = tracer.readLine();

  // 라인 완전 이탈 시: 안전 정지(교육용 기본)
  if (line.lost) {
    tracer.stop();
    return;
  }

  // (2) 오차 계산
  int32_t error = tracer.computeError(line.position);

  // (3) PID 보정값 계산
  int32_t correction = tracer.computeCorrection(error);

  // (4) 모터 적용
  tracer.drive(BASE_SPEED, correction, MAX_SPEED);

  // 디버그 출력(선택): 너무 자주 출력하면 제어가 흔들릴 수 있어 200ms 주기
  // static uint32_t last = 0;
  // if (Serial && (millis() - last >= 200)) {
  //   last = millis();
  //   Serial.print("pos="); Serial.print(line.position);
  //   Serial.print(" err="); Serial.print(error);
  //   Serial.print(" corr="); Serial.println(correction);
  // }
}
