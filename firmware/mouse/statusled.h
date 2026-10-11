#pragma once
#include <Arduino.h>
#include "esp32-hal-rgb-led.h"
#include "config.h"

// 板载 WS2812 状态灯（GPIO48）。亮度压低，晃眼无意义。
// 注：若本板 LED 颗粒颜色顺序不是 GRB，颜色会对调/错位，仅影响观感不影响功能。
inline void ledRGB(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(PIN_LED, r, g, b); }

// 主循环按状态合成 LED：
//   无传感器=红灯闪  回中=绿闪  点击冻结=暗黄  跟踪=暗青  静止冻结=灭  待机=暗蓝
enum LedShow : uint8_t {
  LED_BOOT, LED_ERR, LED_RCFLASH, LED_CLICK, LED_TRACK, LED_FROZEN, LED_IDLE, LED_MAINT
};
inline void ledShow(LedShow s, uint32_t nowMs, uint32_t rcTime) {
  bool blink = (nowMs / 400) & 1;
  switch (s) {
    case LED_ERR:    ledRGB(blink ? 24 : 0, 0, 0); break;
    case LED_RCFLASH:ledRGB(0, 48, 0); break;                    // 回中后闪 RECENTER_FLASH_MS
    case LED_CLICK:  ledRGB(16, 12, 0); break;
    case LED_TRACK:  ledRGB(0, 10, 12); break;
    case LED_FROZEN: ledRGB(0, 0, 0); break;
    case LED_BOOT:   ledRGB(8, 8, 0); break;
    case LED_MAINT:  ledRGB(6, 6, 6); break;                     // 非 abs 模式（log/rate/idle）
    case LED_IDLE:   ledRGB(0, 0, 8); break;
  }
  (void)rcTime;
}
