#include <RiaLineTracerR4.h>

// Sensor pins (맨 왼쪽 라인 검출 센서가 연결된 핀(4번)부터 입력)
const uint8_t sensorPins[] = { 4, A3, 11, A0, A2, 5 };
RiaLineTracerR4 tracer(sensorPins, 6);  // emitterPin default=2

void setup() {
  Serial.begin(115200);
  tracer.begin(true);                   // 센서/모터/이미터 초기화
}

void loop() {
  uint16_t sensors_raw_value[tracer.sensorCount()];          // 6개 각각의 라인 검출 센서 값을 저장하기 위한 변수
  tracer.readRaw(sensors_raw_value);    // 라인 검출 센서 6개로부터 Raw 값 읽기(us)
  
  for (int i = 0; i < tracer.sensorCount(); i++) {
    Serial.print(raw[i]);
    if (i < tracer.sensorCount() - 1) Serial.print('\t');
  }
  Serial.println();
  delay(100);
}
