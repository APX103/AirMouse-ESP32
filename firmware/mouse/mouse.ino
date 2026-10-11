/*
 * AirMouse-Wireless 鼠标端（板 B + ICM20948）v0.2
 *
 * v0.2 变更：
 *   - 时间关键管线（按键→IMU→融合→指针→无线）跑在核心1的高优先级 FreeRTOS 任务，
 *     串口 shell/LED 留在低优先级主循环 → 采样不再被串口打印打断
 *   - CPU 240→160MHz + WiFi 发射功率 19.5→10dBm（降温省电；融合只需 0.02ms）
 *   - 重锚定加投影守卫+限幅（修"一边有墙"），driftComp 默认开启
 *
 * IDE：板型 ESP32S3 Dev Module（无 USB 外设依赖；串口走 UART0/CH340）
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
static uint8_t lastCal = 0;
static volatile uint32_t lastRecenterT = 0;

// 指针输出 → 无线链路（在管线任务上下文里调用）
static void onMove(int dx, int dy) { airlink.sendDelta(dx, dy, btnBits); }
static void onWarp(uint16_t x, uint16_t y) { airlink.sendWarp(x, y); }

// ============ 时间关键管线：核心1 高优先级 FreeRTOS 任务 ============
static void pipelineTask(void *) {
  uint32_t lastHbT = 0;
  for (;;) {
    // 按键语义（防抖在 5ms 扫描周期内完成）
    ButtonEvent ev;
    buttons.poll(ev);
    if (ev.leftDown)  { btnBits |=  AIRLINK_BTN_L; pointer.clickFreeze(); pointer.clickSetTimer(millis()); }
    if (ev.leftUp)    { btnBits &= ~AIRLINK_BTN_L; pointer.clickUnfreeze(); }
    if (ev.rightDown) { btnBits |=  AIRLINK_BTN_R; pointer.clickFreeze(); pointer.clickSetTimer(millis()); }
    if (ev.rightUp)   { btnBits &= ~AIRLINK_BTN_R; pointer.clickUnfreeze(); }
    if (ev.recenter)  { pointer.recenter(); lastRecenterT = millis(); }

    // 传感器 → 指针管线
    Quat q;
    uint8_t cal = 0;
    if (imu.poll(q, cal)) {
      lastQ = q; lastCal = cal;
      pointer.feedGyro(imu.gyroDps[0], imu.gyroDps[1], imu.gyroDps[2]);
      uint32_t t = millis();
      switch (params.mode) {
        case MODE_ABS:  pointer.onQuat(q, t); break;
        case MODE_RATE: pointer.onQuatRate(q, t); break;
        default:        pointer.onQuatLog(q, t); break;
      }
    }
    if (pointer.clickFrozen() && pointer.dragArmed()) pointer.clickUnfreeze();

    // 心跳：500ms 一发，携带绝对模型坐标让接收端纠偏
    uint32_t now = millis();
    if (now - lastHbT >= 500) {
      lastHbT = now;
      airlink.sendHeartbeat(pointer.modelX(), pointer.modelY(), btnBits);
    }
    vTaskDelay(1);  // 1ms 节拍让同核低优先级任务运行；ICM 门控粒度即 1ms
  }
}

void setup() {
  setCpuFrequencyMhz(160);           // 降温：融合计算 0.02ms，160MHz 绰绰有余
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

  Serial.printf("\n=== AirMouse-WL 鼠标端 v0.2 (板B+IMU) | CPU=%uMHz | MAC=%s ===\n",
                getCpuFrequencyMhz(), WiFi.macAddress().c_str());
  if (imuOk) {
    Serial.printf("%s @0x%02X OK。板A接收端上电即自动配对。\n",
                  params.sensor == SENSOR_BNO085 ? "BNO085" : "ICM20948",
                  params.i2cAddr);
  } else {
    Serial.println("!! IMU 未找到：检查接线后 scan；addr 0x68/0x69 可试。");
  }
  Serial.println("回中=BOOT键；串口 help 查看命令。");

  // 管线任务：核心1（与WiFi所在的核0隔离），优先级20（Arduino主循环为1）
  xTaskCreatePinnedToCore(pipelineTask, "pipe", 8192, nullptr, 20, nullptr, 1);
}

void loop() {
  shell.poll();

  // ---- 遥测 CSV（低优先级，不影响采样）----
  static uint32_t tTel = 0;
  uint32_t now = millis();
  if (shell.logEnabled && now - tTel >= 50) {
    tTel = now;
    Quat q = lastQ;
    float yaw, pit, roll;
    toEulerDeg(q, yaw, pit, roll);
    Serial.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.2f,%d,%u\n",
                  now, q.w, q.x, q.y, q.z,
                  yaw, pit, roll,
                  pointer.speedEma, pointer.frozen ? 1 : 0, lastCal);
  }

  // ---- 状态灯（叠加链路健康）----
  static uint32_t tLed = 0;
  if (now - tLed >= 50) {
    tLed = now;
    LedShow s;
    if (!imu.ok())                                    s = LED_ERR;
    else if (!airlink.healthy())                      s = LED_ERR;
    else if (now - lastRecenterT < RECENTER_FLASH_MS) s = LED_RCFLASH;
    else if (params.mode != MODE_ABS)                 s = LED_MAINT;
    else if (pointer.clickFrozen())                   s = LED_CLICK;
    else if (pointer.frozen)                          s = LED_FROZEN;
    else if (pointer.speedEma > MOTION_DPS)           s = LED_TRACK;
    else                                              s = LED_IDLE;
    ledShow(s, now, lastRecenterT);
  }
}
