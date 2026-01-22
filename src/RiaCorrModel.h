#pragma once
#include <Arduino.h>

// Auto-generated from Ridge export JSON
// Input: calibrated sensor values cal[0..N-1] (0..1000)
// Output: correction (int32) with clamp

namespace RiaCorrModel {

static constexpr int N = 6;

static constexpr float MEAN[N] = {
  102.46612f, 152.8486f, 391.39977f, 392.5662f,
  162.31982f, 102.56223f
};

static constexpr float SCALE[N] = {
  261.63803f, 263.36893f, 274.21024f, 287.29091f,
  262.26036f, 254.73129f
};

static constexpr float COEF[N] = {
  -68.50264f, -56.280804f, -17.922413f, 6.3936052f,
  35.825916f, 97.985771f
};

static constexpr float INTERCEPT = 4.1959181f;

// clamp helper
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Predict correction from calibrated sensors
// clampAbs: recommended 80~120 (start with 80)
static inline int32_t predictCorr(const uint16_t* cal, int32_t clampAbs = 80) {
  float y = INTERCEPT;

  for (int i = 0; i < N; i++) {
    // StandardScaler: (x - mean) / scale
    float x = ( (float)cal[i] - MEAN[i] ) / SCALE[i];
    y += x * COEF[i];
  }

  // round to int
  int32_t yi = (y >= 0.0f) ? (int32_t)(y + 0.5f) : (int32_t)(y - 0.5f);

  // safety clamp
  yi = clamp_i32(yi, -clampAbs, +clampAbs);
  return yi;
}

} // namespace RiaCorrModel
