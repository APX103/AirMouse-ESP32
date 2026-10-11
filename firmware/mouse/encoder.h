#pragma once
#include <Arduino.h>
#include "config.h"

// 微雪增量旋转编码器（20 脉冲/圈 + 按键）—— 灵敏度旋钮
// 解码：1kHz 轮询，A 相下降沿 = 一格，此刻读 B 相定方向；
//      连续 2 拍一致才认电平（消机械抖动）；A 须回高 ≥2 拍才接受下一次下降
class Encoder {
public:
  void begin() {
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
  }

  // 每管线节拍(≈1ms)调用。返回本拍净格数(-1/0/+1)；pressed=true 表示刚按下
  int8_t poll(bool &pressed) {
    pressed = false;
    uint8_t a = digitalRead(PIN_ENC_A);
    uint8_t b = digitalRead(PIN_ENC_B);

    // A 相消抖 + 边沿检测
    if (a == aStable_) {
      if (aDeb_ < 255) aDeb_++;
    } else {
      if (a == aLastRaw_ && ++aRawCnt_ >= 2) {   // 连续2拍新电平 → 采纳
        aStable_ = a; aDeb_ = 0;
        if (aStable_ == 0 && aHighRun_ >= 2) {    // 高电平维持过后出现下降沿 = 一格
          // 方向：若反了，交换 SIA/SIB 两根线或把 ENC_REVERSE 改 1
          int8_t dir = (b == HIGH) ? 1 : -1;
#if ENC_REVERSE
          dir = -dir;
#endif
          net_ += dir;
        }
      } else if (a != aLastRaw_) {
        aLastRaw_ = a; aRawCnt_ = 1;
      }
    }
    aHighRun_ = (aStable_ == HIGH) ? (aDeb_ > 2 ? 3 : aHighRun_ + 1) : 0;

    // SW 消抖（低电平=按下）
    uint8_t sw = digitalRead(PIN_ENC_SW);
    if (sw == swStable_) {
      swCnt_ = 0;
    } else if (++swCnt_ >= 20) {   // ~20ms
      swStable_ = sw; swCnt_ = 0;
      if (sw == LOW) pressed = true;
    }

    int8_t out = (int8_t)net_;
    net_ = 0;
    return out;
  }

private:
  uint8_t aStable_ = HIGH, aLastRaw_ = HIGH, aRawCnt_ = 0, aDeb_ = 0, aHighRun_ = 3;
  uint8_t swStable_ = HIGH, swCnt_ = 0;
  int8_t net_ = 0;
};
