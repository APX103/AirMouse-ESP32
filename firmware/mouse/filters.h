#pragma once
#include <Arduino.h>
#include "quat.h"

// One-Euro Filter：低速强低通（稳），高速弱低通（跟手）。参数 fc_min / beta。
struct OneEuro {
  float fcMin = 1.0f, beta = 0.007f, dCutoff = 1.0f;
  float xPrev = 0, dPrev = 0;
  uint32_t tPrev = 0;
  bool init = false;

  void setParams(float fc, float b) { fcMin = fc; beta = b; }
  void snap(float x) {              // 冻结/回中后重置状态，防止恢复瞬间跳变
    xPrev = x; dPrev = 0; init = true; tPrev = millis();
  }
  static float alphaFor(float fc, float dt) {
    float r = 2.0f * Q_PI * fc * dt;
    return r / (r + 1.0f);
  }
  float filter(float x, uint32_t tMs) {
    if (!init) { snap(x); return x; }
    float dt = (tMs - tPrev) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;
    tPrev = tMs;
    float dx = (x - xPrev) / dt;
    float aD = alphaFor(dCutoff, dt);
    float dHat = aD * dx + (1.0f - aD) * dPrev;
    dPrev = dHat;
    float cutoff = fcMin + beta * fabsf(dHat);
    float a = alphaFor(cutoff, dt);
    float xHat = a * x + (1.0f - a) * xPrev;
    xPrev = x;
    return xHat;
  }
};
