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

│  ├─ riaLineTracerR4.h

│  └─ riaLineTracerR4.cpp

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
