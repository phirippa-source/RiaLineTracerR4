#include <RiaLineTracerR4.h>

const uint8_t sensorPins[] = { 5, A2, A0, 11, A3, 4 };
RiaLineTracerR4 tracer(sensorPins, 6); // emitterPin 기본값 2

void setup() {
  Serial.begin(115200);

  tracer.begin(true);               // true : emitter 항상 ON, false : 라인센서 읽을 때만 on
  tracer.calibrate(200, 2500, 10, 5);

  tracer.setPID(0.18f, 0.0f, 1.2f);
  tracer.setIntegralLimit(1500.0f);

  tracer.setSearchOnLost(true);
  tracer.setSearchTurn(90);

  // 모터가 저속에서 안 돌면:
  // tracer.setMinPwmWhenMoving(40);
}

void loop() {
  auto cmd = tracer.step(140, 220, false, 2500, 10, 50);

  static uint32_t last = 0;
  if (millis() - last > 100) {
    last = millis();
    Serial.print("lost="); Serial.print(cmd.lineLost);
    Serial.print(" pos="); Serial.print(cmd.position);
    Serial.print(" err="); Serial.print(cmd.error);
    Serial.print(" L="); Serial.print(cmd.left);
    Serial.print(" R="); Serial.println(cmd.right);
  }
}
