# RiaLineTracerR4

A stable line-tracing library for **Arduino UNO R4 WiFi** + **Zumo Reflectance Sensor Array (RC timing type)**.

This library includes:
- Fast RC timing sensor read (early-exit + pending mask)
- Calibration (min/max) + normalization (0..1000)
- Weighted line position:
  - `readLine()`
  - `readLineWithStatus()` (returns `lineLost`)
- PID helper (anti-windup)
- PWM motor drive as **member functions**
- **Default emitterPin = 2**

---

## Target Platform

- Arduino UNO R4 WiFi (`architectures=renesas_uno`)

---

## Manual Install (Arduino IDE)

Copy this folder to your Arduino libraries path:

- **Windows**: `C:\Users\<YOU>\Documents\Arduino\libraries\RiaLineTracerR4\`
- **macOS**: `~/Documents/Arduino/libraries/RiaLineTracerR4/`
- **Linux**: `~/Arduino/libraries/RiaLineTracerR4/`

Folder layout:

```
RiaLineTracerR4/
├─ library.properties
├─ README.md
└─ src/
   ├─ riaLineTracerR4.h
   └─ riaLineTracerR4.cpp
```

Restart Arduino IDE after copying.

---

## Hardware Assumptions

### Reflectance Sensor (RC timing)
This library is designed for **RC timing type** reflectance sensor arrays.
It performs:
1) charge (set pins OUTPUT/HIGH),
2) switch to INPUT,
3) measure discharge time until LOW.

### Sensor Pins (your mapping)
Example mapping used in this project:

```cpp
const uint8_t sensorPins[] = { 5, A2, A0, 11, A3, 4 };
```

### Emitter Pin
- **Default emitterPin is `2`**.
- If your emitter is wired differently, pass your emitter pin explicitly:
  - `RiaLineTracerR4 tracer(sensorPins, 6, <yourEmitterPin>);`

### Motor Pins (default Zumo Shield style)
Default motor pins are set like this:

- Left DIR: `D8`
- Left PWM: `D10`
- Right DIR: `D7`
- Right PWM: `D9`

If your wiring differs, call:

```cpp
tracer.setMotorPins(leftDir, leftPwm, rightDir, rightPwm);
```

Direction polarity default is **LOW=forward, HIGH=reverse**.
If your motors run reversed, you can flip it:

```cpp
tracer.setDirPolarity(false);
```

---

## Quick Start Example (Minimal)

```cpp
#include <RiaLineTracerR4.h>

// Your sensor pin mapping
const uint8_t sensorPins[] = { 5, A2, A0, 11, A3, 4 };

// Default emitterPin = 2, so you can omit it:
RiaLineTracerR4 tracer(sensorPins, 6);

void setup() {
  Serial.begin(115200);

  // emitterAlwaysOn=true: emitter stays ON
  tracer.begin(true);

  // Calibration:
  // Move the robot over BOTH the line and the background during calibration.
  tracer.calibrate(
    200,   // iterations
    2500,  // timeoutMicros (typical: 2000~3000us)
    10,    // chargeMicros
    5      // interDelayMs
  );

  // PID initial values (MUST tune for your robot)
  tracer.setPID(0.18f, 0.0f, 1.2f);
  tracer.setIntegralLimit(1500.0f);

  // Optional: line-lost behavior
  tracer.setSearchOnLost(true);
  tracer.setSearchTurn(90);

  // Optional: overcome motor dead-zone (try 30~60)
  // tracer.setMinPwmWhenMoving(40);
}

void loop() {
  // step() runs:
  //  read line -> PID -> mix -> applyMotor(PWM)
  auto cmd = tracer.step(
    140,   // baseSpeed
    220,   // maxSpeed (<=255 recommended)
    false, // whiteLine (false: black line on white background)
    2500,  // timeoutMicros
    10,    // chargeMicros
    50     // noiseThreshold
  );

  // Debug print (rate-limited)
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

---

## API Summary

### Core
- `begin(emitterAlwaysOn=true)`
- `calibrate(iterations, timeoutMicros, chargeMicros, interDelayMs)`
- `step(baseSpeed, maxSpeed, whiteLine, timeoutMicros, chargeMicros, noiseThreshold)`

### Sensor
- `readRaw(values, timeoutMicros, chargeMicros)` -> discharge time (us)
- `readCalibrated(values, timeoutMicros, chargeMicros)` -> 0..1000
- `readLineWithStatus(lineLost, ...)` -> position + lineLost

### PID
- `setPID(kp, ki, kd)`
- `setIntegralLimit(limitAbs)`
- `resetPID()`

### Motor
- `applyMotor(left, right)` -> PWM + DIR applied
- `setMotorPins(...)`
- `setDirPolarity(lowIsForward)`
- `setMinPwmWhenMoving(minPwm)`

---

## Practical Tuning Notes

1) Tune in this order: **P -> PD -> (optional) I**.
2) If sensor read time is too slow, reduce `timeoutMicros` (start from 2000~3000us).
3) Adjust `noiseThreshold` to reduce false `lineLost` detection.
4) If the robot does not move at low speeds, increase `setMinPwmWhenMoving(30~60)`.

---

## License

MIT (or replace with your preferred license).
