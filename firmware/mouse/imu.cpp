#include "imu.h"

// I2C INT 中断（仅 BNO085 用；文件作用域，避免头文件内联 IRAM 导致链接错误）
static Imu *s_imuForIsr = nullptr;
static void ARDUINO_ISR_ATTR imuIsr() {
  if (s_imuForIsr) s_imuForIsr->intFlag = true;
}

bool Imu::begin(uint8_t sensorType, uint8_t addr, uint32_t wireHz) {
  ok_ = false;
  type_ = sensorType;
  addr_ = addr;
  s_imuForIsr = this;

  Wire.begin(PIN_SDA, PIN_SCL, wireHz);
  Wire.setTimeOut(50);

#ifndef AIM_HAS_BNO085
  if (type_ == SENSOR_BNO085) {
    Serial.println(F("[IMU] 未编译 BNO085 支持（缺 Adafruit BNO08x 库）"));
    return false;
  }
#endif
  bool r = (type_ == SENSOR_BNO085) ? beginBno() : beginIcm();
  ok_ = r;
  return r;
}

void Imu::setReportHz(uint32_t hz) {
  uint32_t cap = (type_ == SENSOR_BNO085) ? 400 : 250;
  if (hz < 25) hz = 25;
  if (hz > cap) hz = cap;
  hz_ = hz;
#ifdef AIM_HAS_BNO085
  if (type_ == SENSOR_BNO085 && ok_) {
    bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 1000000UL / hz);
  }
#endif
  // ICM：门控周期（µs）
  gateUs_ = 1000000UL / hz;
}

// ---------------- BNO085（片上 GRV 融合；需要 Adafruit BNO08x 库）----------------
#ifdef AIM_HAS_BNO085
bool Imu::beginBno() {
  // 硬复位：RST 拉低 ≥10ms，释放后等芯片起机
  pinMode(PIN_IMU_RST, OUTPUT);
  digitalWrite(PIN_IMU_RST, LOW);
  delay(10);
  digitalWrite(PIN_IMU_RST, HIGH);
  delay(150);

  if (!bno_.begin_I2C(addr_, &Wire)) return false;
  setReportHz(hz_);

  pinMode(PIN_IMU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuIsr, FALLING);
  intFlag = false;
  return true;
}

void Imu::pollBno(Quat &q, uint8_t &calStatus, bool &out) {
  uint32_t now = millis();
  bool due = intFlag;
  if (!due && now - lastPollMs_ >= 2) due = true; // INT 丢失兜底
  if (!due) return;
  intFlag = false;
  lastPollMs_ = now;

  sh2_SensorValue_t ev;
  if (!bno_.getSensorEvent(&ev)) return;
  if (bno_.wasReset()) { setReportHz(hz_); return; }
  if (ev.sensorId != SH2_GAME_ROTATION_VECTOR) return;

  q = qnorm(Quat{ev.un.gameRotationVector.real,
                 ev.un.gameRotationVector.i,
                 ev.un.gameRotationVector.j,
                 ev.un.gameRotationVector.k});
  calStatus = ev.status;
  lastCal = calStatus;
  lastOkMs_ = now;
  out = true;
}
#endif // AIM_HAS_BNO085

// ---------------- ICM20948（SparkFun 裸读 + 板载 Madgwick）----------------
bool Imu::beginIcm() {
  bool ad0High = (addr_ == 0x69); // 微雪板 AD0 接地 → 0x68
  if (icm_.begin(Wire, ad0High) != ICM_20948_Stat_Ok) return false;

  // 量程：陀螺 ±1000dps（快速甩动不削顶），加计 ±4g
  ICM_20948_fss_t fss;
  fss.a = 1; // ±4g
  fss.g = 2; // ±1000dps
  icm_.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  // 使能数字低通（默认档位，兼顾平滑与延迟）
  icm_.enableDLPF(ICM_20948_Internal_Gyr, true);
  icm_.enableDLPF(ICM_20948_Internal_Acc, true);

  setReportHz(hz_);
  calGyroBiasIcm(); // 静置 1.5s 标零偏（期间别动）
  madg_ = Madgwick6{}; // 重置融合器，等待首个重力向量初始化
  lastIcmUs_ = micros();
  return true;
}

void Imu::calGyroBiasIcm() {
  Serial.println(F("[IMU] ICM20948 陀螺零偏标定中，请保持静止…"));
  const int N = 150;
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N; i++) {
    icm_.getAGMT();
    sx += icm_.gyrX(); sy += icm_.gyrY(); sz += icm_.gyrZ();
    delay(10);
  }
  gBias[0] = sx / N; gBias[1] = sy / N; gBias[2] = sz / N;
  Serial.printf("[IMU] 零偏 %.2f %.2f %.2f dps\n", gBias[0], gBias[1], gBias[2]);
  lastCal = 3;
}

void Imu::pollIcm(Quat &q, bool &out) {
  uint32_t nowUs = micros();
  if (nowUs - lastIcmUs_ < gateUs_) return; // 门控到设定频率
  lastIcmUs_ = nowUs;

  icm_.getAGMT();
  gyroDps[0] = icm_.gyrX() - gBias[0];   // °/s，去偏（供积分模式直接使用）
  gyroDps[1] = icm_.gyrY() - gBias[1];
  gyroDps[2] = icm_.gyrZ() - gBias[2];
  gyroMagDps = sqrtf(gyroDps[0]*gyroDps[0] + gyroDps[1]*gyroDps[1] + gyroDps[2]*gyroDps[2]);
  float gx = gyroDps[0] * DEG2RAD;
  float gy = gyroDps[1] * DEG2RAD;
  float gz = gyroDps[2] * DEG2RAD;
  float ax = icm_.accX() / 1000.0f; // milli-g → g
  float ay = icm_.accY() / 1000.0f;
  float az = icm_.accZ() / 1000.0f;
  q = madg_.update(gx, gy, gz, ax, ay, az, millis());
  lastOkMs_ = millis();
  out = true;
}

bool Imu::poll(Quat &q, uint8_t &calStatus) {
  if (!ok_) return false;
  bool out = false;
#ifdef AIM_HAS_BNO085
  if (type_ == SENSOR_BNO085) pollBno(q, calStatus, out);
  else
#endif
  { pollIcm(q, out); if (out) calStatus = lastCal; }
  return out;
}
