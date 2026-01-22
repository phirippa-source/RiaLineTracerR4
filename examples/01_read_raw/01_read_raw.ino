#include <RiaLineTracerR4.h>

// Sensor pins (맨 왼쪽 라인 검출 센서가 연결된 핀(4번)부터 입력)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin default=2

void setup() {
  Serial.begin(115200);

  // 1) 센서/모터/이미터 초기화
  tracer.begin(true);

  Serial.println("01_read_raw READY");
}

void loop() {
  // 2) Raw 읽기(us)
  uint16_t raw[8];
  tracer.readRaw(raw);

  // 3) 출력(너무 자주 출력하면 느려지므로 100ms에 1번)
  static uint32_t last = 0;
  if (millis() - last >= 100) {
    last = millis();
    for (int i = 0; i < 6; i++) {
      Serial.print(raw[i]);
      if (i < 5) Serial.print('\t');
    }
    Serial.println();
  }
}
