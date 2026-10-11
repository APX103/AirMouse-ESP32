#pragma once
#include <Arduino.h>
#include "config.h"

// 三路按键（左/右/回中）防抖 + 沿检测（自有线版，去掉 USB 依赖）
struct ButtonEvent {
  bool leftDown = false, leftUp = false;
  bool rightDown = false, rightUp = false;
  bool recenter = false;
};

struct Debouncer {
  bool state = false;       // true = 按下（低电平）
  bool update(bool raw, uint32_t now, uint32_t debMs = 20) {
    if (raw == state) { tPend = 0; return false; }
    if (tPend == 0 || raw != pend) { pend = raw; tPend = now; return false; }
    if (now - tPend >= debMs) { state = raw; tPend = 0; return true; }
    return false;
  }
private:
  bool pend = false; uint32_t tPend = 0;
};

class Buttons {
public:
  void begin() {
    pinMode(PIN_BTN_L, INPUT_PULLUP);
    pinMode(PIN_BTN_R, INPUT_PULLUP);
    pinMode(PIN_BTN_RC, INPUT_PULLUP);
  }
  void poll(ButtonEvent &ev) {
    ev = ButtonEvent{};
    uint32_t now = millis();
    if (now - lastPoll_ < 5) return;
    lastPoll_ = now;
    if (debL.update(digitalRead(PIN_BTN_L) == LOW, now))
      debL.state ? ev.leftDown = true : ev.leftUp = true;
    if (debR.update(digitalRead(PIN_BTN_R) == LOW, now))
      debR.state ? ev.rightDown = true : ev.rightUp = true;
    if (debRc.update(digitalRead(PIN_BTN_RC) == LOW, now))
      if (debRc.state) ev.recenter = true;
  }
private:
  Debouncer debL, debR, debRc;
  uint32_t lastPoll_ = 0;
};
