#pragma once
#include <Wire.h>
#include <ICM_20948.h>
#include "quat.h"
#include "config.h"
#include "madgwick.h"

// 装了 Adafruit BNO08x 库才编译 BNO085 支持（没装也能编，只用 ICM20948）
#if __has_include(<Adafruit_BNO08x.h>)
#include <Adafruit_BNO08x.h>
#define AIM_HAS_BNO085 1
#endif

// 双传感器驱动，统一接口：
//   SENSOR_BNO085  —— Adafruit 库，片上 GRV 融合（原方案）
//   SENSOR_ICM20948—— SparkFun 库裸读陀螺/加计 + 板载 Madgwick 6轴融合
//                     （输出与 GRV 同构：pitch/roll 不漂，yaw 慢漂）
class Imu {
public:
  bool begin(uint8_t sensorType, uint8_t addr, uint32_t wireHz);
  void setReportHz(uint32_t hz);       // 25..400（ICM 为融合更新率）
  bool poll(Quat &q, uint8_t &calStatus); // 有新姿态返回 true
  bool ok() const { return ok_; }
  uint32_t reportHz() const { return hz_; }
  uint8_t addr() const { return addr_; }
  uint8_t type() const { return type_; }
  uint8_t lastCal = 0; // 最近校准状态 0..3（ICM 恒 3）

  volatile bool intFlag = false;

private:
  bool beginBno();
  bool beginIcm();
  void calGyroBiasIcm();
  void pollBno(Quat &q, uint8_t &calStatus, bool &out);
  void pollIcm(Quat &q, bool &out);

#ifdef AIM_HAS_BNO085
  Adafruit_BNO08x bno_;
#endif
  ICM_20948_I2C icm_;
  Madgwick6 madg_;
  float gBias[3] = {0, 0, 0};
  bool ok_ = false;
  uint32_t hz_ = 100;
  uint8_t addr_ = 0x68;
  uint8_t type_ = SENSOR_ICM20948;
  uint32_t lastPollMs_ = 0;
  uint32_t lastIcmUs_ = 0;
  uint32_t lastOkMs_ = 0;
  uint32_t gateUs_ = 10000; // ICM 轮询门控周期（µs），由 setReportHz 重算
};
