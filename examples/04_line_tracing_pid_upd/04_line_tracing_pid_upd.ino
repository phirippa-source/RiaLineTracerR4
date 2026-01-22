#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <RiaLineTracerR4.h>

// ==============================
// Wi-Fi / UDP 설정
// ==============================
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

IPAddress PC_IP(192, 168, 0, 23);   // ipconfig의 PC IPv4로 변경
const uint16_t PC_PORT = 5005;

WiFiUDP Udp;

// ==============================
// Sensor pins (LEFT -> RIGHT)
// ==============================
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

// ==============================
// (수업에서 보여야 하는 핵심 파라미터)
// ==============================
static constexpr float KP = 0.08f;
static constexpr float KI = 0.0f;
static constexpr float KD = 0.01f;

static constexpr int16_t BASE_SPEED = 170;
static constexpr int16_t MAX_SPEED  = 220;

static constexpr uint16_t START_DELAY_MS = 1500;

// ==============================
// UDP 송신 주기 제어(권장 50~100Hz)
// ==============================
static constexpr uint16_t SEND_PERIOD_MS = 10; // 10ms = 100Hz
uint32_t lastSendMs = 0;

uint32_t seq = 0;
uint32_t lastLoopUs = 0;

void connectWiFiSimple() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // 질문에서 말씀하신 방식: localIP 호출 전 약간 delay
  delay(500);

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("UNO IP: ");
  Serial.println(WiFi.localIP());
}

void sendTelemetryCSV(uint32_t seq,
                      uint32_t t_ms,
                      uint32_t dt_us,
                      bool lost,
                      int32_t pos,
                      int32_t err,
                      int32_t corr,
                      int16_t baseSpd,
                      int16_t maxSpd,
                      int16_t L,
                      int16_t R,
                      const uint16_t* cal6,
                      uint8_t n)
{
  // 충분히 크게 잡기
  char buf[256];

  // cal은 sensorCount()만큼만 출력
  // 포맷: seq,t_ms,dt_us,lost,pos,err,corr,base,max,L,R,cal0..cal5
  // n=6이면 cal0..cal5
  int len = snprintf(buf, sizeof(buf),
    "%lu,%lu,%lu,%u,%ld,%ld,%ld,%d,%d,%d,%d",
    (unsigned long)seq,
    (unsigned long)t_ms,
    (unsigned long)dt_us,
    (unsigned)lost,
    (long)pos,
    (long)err,
    (long)corr,
    (int)baseSpd,
    (int)maxSpd,
    (int)L,
    (int)R
  );

  // cal append
  for (uint8_t i = 0; i < n; i++) {
    len += snprintf(buf + len, sizeof(buf) - len, ",%u", (unsigned)cal6[i]);
  }

  Udp.beginPacket(PC_IP, PC_PORT);
  Udp.write((const uint8_t*)buf, strlen(buf));
  Udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(800);

  // 1) Wi-Fi 연결
  connectWiFiSimple();
  Udp.begin(0);

  // 2) 라인트레이서 초기화
  tracer.begin(true);

  delay(START_DELAY_MS);

  // 3) 캘리브레이션(기본값 사용)
  tracer.calibrateSpin();

  // 4) PID 설정
  tracer.setPID(KP, KI, KD);
  tracer.setIntegralLimit(1500.0f);
  tracer.resetPID();

  Serial.println("04_line_tracing_pid_udp READY");
}

void loop() {
  // 루프 주기 측정(us)
  const uint32_t nowUs = micros();
  const uint32_t dtUs = (lastLoopUs == 0) ? 0 : (nowUs - lastLoopUs);
  lastLoopUs = nowUs;

  // 송신 주기 제한(10ms마다)
  const uint32_t nowMs = millis();
  if (nowMs - lastSendMs < SEND_PERIOD_MS) {
    // 주행은 계속하되, 텔레메트리는 주기적으로만 전송
  } else {
    lastSendMs = nowMs;
  }

  // (1) 센서 1회 읽기: cal + line 동시에 얻기
  uint16_t cal[RiaLineTracerR4::MAX_SENSORS];
  auto line = tracer.readLineWithCal(cal); // 기본값 사용

  // (2) 라인 이탈 처리(교육용 안전)
  if (line.lost) {
    tracer.stop();

    // 송신 주기 맞을 때만 로그
    if (nowMs == lastSendMs) {
      sendTelemetryCSV(seq++, nowMs, dtUs, true, line.position, 0, 0,
                       BASE_SPEED, MAX_SPEED, 0, 0, cal, tracer.sensorCount());
    }
    return;
  }

  // (3) 오차
  int32_t err = tracer.computeError(line.position);

  // (4) PID 보정
  int32_t corr = tracer.computeCorrection(err);

  // (5) 모터 적용(+실제 L/R 확보)
  auto m = tracer.driveWithReport(BASE_SPEED, corr, MAX_SPEED);

  // (6) 송신 주기 맞을 때만 텔레메트리 전송
  if (nowMs == lastSendMs) {
    sendTelemetryCSV(seq++, nowMs, dtUs, false, line.position, err, corr,
                     BASE_SPEED, MAX_SPEED, m.left, m.right,
                     cal, tracer.sensorCount());
  }
}
