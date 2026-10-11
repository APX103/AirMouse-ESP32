#pragma once
#include <Arduino.h>
#include <stdint.h>

// ============ 引脚映射（WeAct ESP32-S3 B 板，见《AirMouse-调研与总体方案》§3）============
// 避开：GPIO19/20(原生USB) GPIO26-32(Flash) GPIO33-37(PSRAM) GPIO48本身可用但接了板载LED
#define PIN_SDA     41   // I2C SDA
#define PIN_SCL     42   // I2C SCL
#define PIN_IMU_INT 40   // BNO085 INT（下降沿=报告就绪；不接也能跑，靠 2ms 轮询兜底）
#define PIN_IMU_RST 38   // BNO085 RST（不走库，自己控制复位时序，兼容任何库版本）
#define PIN_BTN_L   47   // 左键：轻触开关接 GND（内部上拉）
#define PIN_BTN_R   21   // 右键（可选）：开关接 GND
#define PIN_BTN_RC   0   // 回中键 = 板载 BOOT 键（注意：复位瞬间别按着，会进下载模式）
#define PIN_LED     48   // 板载 WS2812 RGB（经 0Ω 电阻）

// ============ 运行模式 ============
enum RunMode : uint8_t { MODE_IDLE = 0, MODE_LOG = 1, MODE_RATE = 2, MODE_ABS = 3 };

// ============ 传感器类型（`sensor` 命令切换，重启生效）============
enum SensorType : uint8_t { SENSOR_BNO085 = 0, SENSOR_ICM20948 = 1 };

// ============ 安装朝向：设备"前向"对应 IMU 哪个轴（`mount` 命令现场标定）============
enum FwdAxis : uint8_t { FWD_PX = 0, FWD_NX, FWD_PY, FWD_NY, FWD_PZ, FWD_NZ };

// ============ 可调参数（shell 热改 + NVS 掉电保存；默认值 = 方案文档 §5.8）============
struct Params {
  // 指针映射
  float    fovH      = 75.0f;  // 水平 FOV(°)：±fovH/2 覆盖屏幕宽。要'永不撞墙'就让它
                                // 大于手腕自然转动范围（人腕yaw约±40°，75°即刻意才撞到）
  float    fovV      = 55.0f;  // 垂直 FOV(°)
  uint16_t scrW      = 1920;   // 屏幕分辨率（内部绝对模型用）
  uint16_t scrH      = 1080;
  uint8_t  fwd       = FWD_PY; // 安装朝向，拿到手第一件事就是标定它
  bool     invX      = false;  // 方向反了先试这两个开关，再考虑换轴
  bool     invY      = false;
  // 传感器 / 总线
  uint8_t  sensor    = SENSOR_ICM20948; // bno085 | icm20948（`sensor`命令，重启生效）
  uint8_t  i2cAddr   = 0x68;   // BNO085:0x4A/0x4B  ICM20948:0x68(AD0低)/0x69（重启生效）
  uint32_t i2cHz     = 400000; // 100000 / 400000
  uint32_t reportHz  = 100;    // BNO:GRV报告率 | ICM:融合更新率（25..250/400）
  // 滤波 / 冻结
  float    euroFc    = 1.0f;   // One-Euro 低速截止 Hz
  float    euroBeta  = 0.007f; // 速度系数（越大越跟手、越抖）
  uint16_t deadPx    = 1;      // 静止滞回死区(px)
  uint16_t freezeMs  = 300;    // 点击冻结：按住多久后才允许转成拖拽
  float    freezeDeg = 3.0f;   // 点击冻结：姿态转动超过几度才转拖拽
  bool     driftComp = true;   // 在线 yaw 漂移补偿（v0.2 起默认开）
  bool     softEdge  = false;  // 软边界(tanh)边缘渐近：实测'永远差一点到不了边'手感更差，默认关
  float    rateGain  = 15.0f;  // MODE_RATE：px/deg（M2 验证用）
  uint8_t  mode      = MODE_RATE; // 无限积分模式（用户选定）；abs=ray-cast绝对指点保留可选
};

// ============ 静止检测阈值（先硬编码；够用就不提参数）============
constexpr float    STATIONARY_DPS  = 1.2f; // EMA 角速度低于此 → 认为静止
constexpr float    MOTION_DPS      = 6.0f; // 高于此 → 认为恢复运动
constexpr uint32_t FREEZE_DELAY_MS = 900;  // 静止持续这么久才冻结光标
constexpr uint32_t RECENTER_FLASH_MS = 300;
