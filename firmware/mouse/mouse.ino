/*
 * AirMouse-Wireless 鼠标端（板 B + ICM20948）
 * 全部指针算法在本板：IMU→Madgwick→ray-cast→FOV→冻结/回中
 * 输出不再走 USB，而是 airlink/ESP-NOW 发给接收端（板 A）
 *
 * IDE：板型 ESP32S3 Dev Module（USB 设置随意，无 USB 外设依赖；串口走 UART0/CH340）
 */
#include <Wire.h>
#include "config.h"
#include "quat.h"
#include "imu.h"
#include "pointer.h"
#include "buttons.h"
#include "link.h"
#include "shell.h"
#include "statusled.h"
#include "airlink.h"

static Params params;
static Imu imu;
static Pointer pointer;
static Buttons buttons;
static Link airlink;
static Shell shell;

static uint8_t btnBits = 0;          // AIRLINK_BTN_L|R
static Quat lastQ;
static uint32_t lastRecenterT = 0;
static uint32_t lastHbT = 0;

// 指针输出 → 无线链路
static void onMove(int dx, int dy) { airlink.sendDelta(dx, dy, btnBits); }
static void onWarp(uint16_t x, uint16_t y) { airlink.sendWarp(x, y); }

void setup() {
  Serial.begin(115200);
  buttons.begin();
  ledRGB(8, 8, 0);
  delay(300);

  shell.begin(&params, &imu, &pointer, &airlink);
  shell.loadFromNvs();
  imu.setReportHz(params.reportHz);

  if (!airlink.begin()) {
    Serial.println("!! ESP-NOW 初始化失败（重启再试）");
  }

  pointer.begin(&params);
  pointer.setCallbacks(onMove, onWarp);

  bool imuOk = imu.begin(params.sensor, params.i2cAddr, params.i2cHz);

  Serial.printf("\n=== AirMouse-WL 鼠标端 (板B+IMU) | MAC=%s ===\n",
                WiFi.macAddress().c_str());
  if (imuOk) {
    Serial.printf("%s @0x%02X OK。板A接收端上电即自动配对。\n",
                  params.sensor == SENSOR_BNO085 ? "BNO085" : "ICM20948",
                  params.i2cAddr);
  } else {
    Serial.println("!! IMU 未找到：检查接线后 scan；addr 0x68/0x69 可试。");
  }
  Serial.println("回中=BOOT键；串口 help 查看命令。");
}

void loop() {
  shell.poll();

  // ---- 按键：本地语义（点击冻结）+ 状态进 buttons 位图随包发送 ----
  ButtonEvent ev;
  buttons.poll(ev);
  if (ev.leftDown)  { btnBits |=  AIRLINK_BTN_L; pointer.clickFreeze(); pointer.clickSetTimer(millis()); }
  if (ev.leftUp)    { btnBits &= ~AIRLINK_BTN_L; pointer.clickUnfreeze(); }
  if (ev.rightDown) { btnBits |=  AIRLINK_BTN_R; pointer.clickFreeze(); pointer.clickSetTimer(millis()); }
  if (ev.rightUp)   { btnBits &= ~AIRLINK_BTN_R; pointer.clickUnfreeze(); }
  if (ev.recenter)  { pointer.recenter(); lastRecenterT = millis(); }

  // ---- 传感器 → 指针管线 ----
  Quat q;
  uint8_t cal = 0;
  if (imu.poll(q, cal)) {
    lastQ = q;
    uint32_t t = millis();
    switch (params.mode) {
      case MODE_ABS:  pointer.onQuat(q, t); break;
      case MODE_RATE: pointer.onQuatRate(q, t); break;
      default:        pointer.onQuatLog(q, t); break;
    }
  }
  if (pointer.clickFrozen() && pointer.dragArmed()) pointer.clickUnfreeze();

  // ---- 心跳：500ms 一发，携带绝对模型坐标让接收端纠偏 ----
  uint32_t now = millis();
  if (now - lastHbT >= 500) {
    lastHbT = now;
    airlink.sendHeartbeat(pointer.modelX(), pointer.modelY(), btnBits);
  }

  // ---- 遥测 CSV（M1 联调用）----
  static uint32_t tTel = 0;
  if (shell.logEnabled && now - tTel >= 50) {
    tTel = now;
    float yaw, pit, roll;
    toEulerDeg(lastQ, yaw, pit, roll);
    Serial.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%d,%u\n",
                  now, lastQ.w, lastQ.x, lastQ.y, lastQ.z,
                  yaw, pit, roll,
                  pointer.speedEma, pointer.frozen ? 1 : 0, cal);
  }

  // ---- 状态灯（叠加链路健康）----
  static uint32_t tLed = 0;
  if (now - tLed >= 50) {
    tLed = now;
    LedShow s;
    if (!imu.ok())                                    s = LED_ERR;
    else if (!airlink.healthy())                         s = LED_ERR;
    else if (now - lastRecenterT < RECENTER_FLASH_MS) s = LED_RCFLASH;
    else if (params.mode != MODE_ABS)                 s = LED_MAINT;
    else if (pointer.clickFrozen())                   s = LED_CLICK;
    else if (pointer.frozen)                          s = LED_FROZEN;
    else if (pointer.speedEma > MOTION_DPS)           s = LED_TRACK;
    else                                              s = LED_IDLE;
    ledShow(s, now, lastRecenterT);
  }
}
