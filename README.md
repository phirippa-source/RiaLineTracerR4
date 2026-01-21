# RiaLineTracerR4

Stable line tracing library for **Arduino UNO R4 WiFi** + **Zumo Reflectance Sensor Array (RC timing type)**.

This library combines:
- Fast RC timing sensor read (early-exit)
- Calibration (min/max) + normalization (0..1000)
- Weighted line position (`readLine`, `readLineWithStatus`)
- Small PID helper + line-lost handling

> Target platform
- Arduino UNO R4 WiFi (architecture: renesas_uno)

---

## Folder Layout (Manual Install)

Place this library folder here:

- Windows: `C:\Users\<YOU>\Documents\Arduino\libraries\RiaLineTracerR4\`
- macOS: `~/Documents/Arduino/libraries/RiaLineTracerR4/`
- Linux: `~/Arduino/libraries/RiaLineTracerR4/`

Recommended structure:

RiaLineTracerR4/

├─ library.properties

├─ README.md

├─ src/

  ├─ riaLineTracerR4.h

  └─ riaLineTracerR4.cpp

└─ examples/

   └─ basic_line_trace/
   
      └─ basic_line_trace.ino

After copying, restart Arduino IDE.

---

## Wiring / Sensor Pins

This project assumes a **6-sensor RC timing array** connected to the following pins
(in the same order as your array mapping):

`{ 5, A2, A0, 11, A3, 4 }`

If you have an IR emitter control pin, pass it as `emitterPin`.  
If you do not use/know it, set `emitterPin = -1`.

---

## Quick Start Example
Create an example sketch (or use examples/basic_line_trace/basic_line_trace.ino):

```cpp
#include <RiaLineTracerR4.h>   // library.properties의 includes에 맞춰 설치 시

const uint8_t sensorPins[] = { 5, A2, A0, 11, A3, 4 };
RiaLineTracerR4 tracer(sensorPins, 6, -1);

void setup() {
  Serial.begin(115200);

  tracer.begin(true);

  tracer.calibrate(200, 2500, 10, 5);

  tracer.setPID(0.18f, 0.0f, 1.2f);
  tracer.setIntegralLimit(1500.0f);

  tracer.setSearchOnLost(true);
  tracer.setSearchTurn(90);

  // 모터가 저속에서 안 돌면:
  // tracer.setMinPwmWhenMoving(40);

  // 모터 핀맵이 다르면:
  // tracer.setMotorPins(8,10, 7,9);
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

```
