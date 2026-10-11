#pragma once
#include "quat.h"
#include "filters.h"
#include "config.h"

// 指针算法核心（方案文档 §5）：
//   四元数 → 世界系指向射线 → 与虚拟屏幕平面求交 → FOV 归一化 → 钳制
//   → One-Euro 平滑 → 死区 → 帧间差分 dx/dy（经回调发给 HID）
// 回中 / 静止冻结重锚定 / 在线漂移补偿 / 点击冻结 都在这里。
class Pointer {
public:
  typedef void (*MoveCb)(int dx, int dy);
  typedef void (*WarpCb)(uint16_t x, uint16_t y);

  void begin(Params *p);
  void setCallbacks(MoveCb m, WarpCb w) { moveCb = m; warpCb = w; }

  // 每收到一个 GRV 报告调用一次（MODE_ABS）
  void onQuat(Quat q, uint32_t tMs);
  // MODE_RATE：无限积分模式（陀螺直接积分，见 pointer.cpp）
  void onQuatRate(Quat q, uint32_t tMs);
  void onQuatLog(Quat q, uint32_t tMs) { lastQ = q; (void)tMs; }
  // 喂陀螺原始角速度（°/s 机体系，去偏）——积分模式的角速度来源
  void feedGyro(float wx, float wy, float wz) { gyroBody = {wx, wy, wz}; haveGyro = true; }

  void recenter();                        // 用当前指向重建锚点，光标跳屏幕中心
  // 点击冻结（左/右键按住期间）
  void clickFreeze();
  void clickUnfreeze();
  bool clickFrozen() const { return clickFrozen_; }
  bool dragArmed() const;                 // 按住超时且姿态已大幅移动 → 允许转拖拽
  void clickSetTimer(uint32_t tMs) { clickDownT = tMs; }

  // ---- 状态/遥测（stat 命令与日志直接读）----
  bool  anchored = false, frozen = false;
  float speedEma = 0;      // °/s
  float rawU = 0, rawV = 0;
  float Xf = 0, Yf = 0;
  float driftDps = 0;      // 静止期估计的 yaw 漂移率 °/s
  Quat  lastQ;             // 最新姿态（按键逻辑要用）

  void setFov(float fh, float fv);
  void setScreen(uint16_t w, uint16_t h);
  void resyncFilter() { euroX.snap(sentX); euroY.snap(sentY); }
  int modelX() const { return sentX; }   // 内部绝对模型（无线心跳用）
  int modelY() const { return sentY; }

private:
  void enterFreeze();
  void exitFreeze();
  Vec3 fwdVec() const;
  void  recalcTan() { tanH_ = tanf(par->fovH * 0.5f * DEG2RAD); tanV_ = tanf(par->fovV * 0.5f * DEG2RAD); }

  Params *par = nullptr;
  MoveCb  moveCb = nullptr;
  WarpCb  warpCb = nullptr;

  // 锚点（世界系水平参考方向）
  Vec3  xhat{1, 0, 0};
  float tanH_ = 0.36f, tanV_ = 0.27f;

  // 派生量缓存
  Vec3  dW{};                 // 当前世界系射线
  Quat  prevQ; bool havePrev = false;
  uint32_t lastT = 0, lastMoveT = 0;

  // 冻结/重锚
  Vec3  dEntry{}; bool haveEntry = false;
  Quat  qEntry;  uint32_t entryT = 0, clickDownT = 0;
  bool  clickFrozen_ = false;

  // 无限积分模式（mode rate）
  Vec3  ratePrev{}; bool haveRatePrev = false;
  Vec3  gyroBody{}; bool haveGyro = false;   // 陀螺角速度 °/s（机体系）

  // 输出
  OneEuro euroX, euroY;
  int sentX = 0, sentY = 0;
};
