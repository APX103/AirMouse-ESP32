#include "pointer.h"

void Pointer::begin(Params *p) {
  par = p;
  euroX.setParams(p->euroFc, p->euroBeta);
  euroY.setParams(p->euroFc, p->euroBeta);
  recalcTan();
  anchored = false;
  frozen = false;
  clickFrozen_ = false;
  xhat = Vec3{1, 0, 0};
}

void Pointer::setFov(float fh, float fv) {
  if (par) {
    par->fovH = clampf(fh, 5.0f, 179.0f);
    par->fovV = clampf(fv, 5.0f, 179.0f);
    recalcTan();
  }
}

void Pointer::setScreen(uint16_t w, uint16_t h) {
  if (par) {
    par->scrW = w < 100 ? 100 : w;
    par->scrH = h < 100 ? 100 : h;
  }
}

Vec3 Pointer::fwdVec() const {
  switch (par->fwd) {
    case FWD_PX: return { 1, 0, 0};
    case FWD_NX: return {-1, 0, 0};
    case FWD_PY: return { 0, 1, 0};
    case FWD_NY: return { 0,-1, 0};
    case FWD_PZ: return { 0, 0, 1};
    default:     return { 0, 0,-1}; // FWD_NZ
  }
}

// ---------------- MODE_ABS 主管线 ----------------
void Pointer::onQuat(Quat q, uint32_t tMs) {
  lastQ = q;
  float dt = lastT ? (tMs - lastT) / 1000.0f : 0.01f;
  lastT = tMs;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

  // 角速度估计（°/s，EMA 平滑）
  if (havePrev) {
    float dps = qAngleDeg(prevQ, q) / dt;
    speedEma += 0.25f * (dps - speedEma);
  }
  prevQ = q; havePrev = true;

  dW = qrot(q, fwdVec()); // 世界系指向射线
  if (!anchored) { recenter(); return; }

  // ---- 静止冻结状态机 ----
  if (!frozen) {
    if (speedEma > MOTION_DPS) lastMoveT = tMs;
    if (tMs - lastMoveT > FREEZE_DELAY_MS && speedEma < STATIONARY_DPS) enterFreeze();
  }
  if (frozen) {
    if (!haveEntry) { // 冻结入口：记下当时的射线与时间，供重锚定/漂移估计用
      dEntry = dW; qEntry = q; entryT = tMs; haveEntry = true;
    } else if (par->driftComp && tMs - entryT > 3000) {
      // 静止期在线漂移率估计。守卫：斜仰角（水平投影短）不估计；
      // 静止期真漂移不可能超 0.5°/s，超了判为真运动丢弃；EMA 平滑。
      Vec3 he = horiz(dEntry), hn = horiz(dW);
      if (vlen(he) > 0.35f && vlen(hn) > 0.35f) {
        float dps = yawBetween(he, hn) * RAD2DEG / ((tMs - entryT) / 1000.0f);
        if (fabsf(dps) < 0.5f)
          driftDps += 0.1f * (dps - driftDps);
      }
    }
    return; // 冻结期间不更新光标
  }
  haveEntry = false;
  if (!par->driftComp) driftDps = 0;

  // ---- 在线漂移补偿 ----
  // 符号推导：yaw 漂 +d 时，传感器报告系绕 ẑ 转 +d，任何固定指向的"报告方向"随之转 +d；
  // 要保持"报告射线 vs 锚点"的夹角不变，锚点必须【同向】转 +d（v0.2.2 修：之前写成 -d，
  // 补偿变双重漂移 → 锚点游走 → 水平饱和无 dx）
  if (par->driftComp && driftDps != 0) {
    xhat = rotZ(xhat, driftDps * DEG2RAD * dt);
  }

  // ---- ray-cast：射线与虚拟屏幕平面（x̂ 方向距离 1）求交 ----
  float dxw = vdot(dW, xhat);
  if (dxw > 0.05f) { // 指向屏幕侧；几乎平行/指向背后时保持上一帧位置
    Vec3 er = vcross(Vec3{0, 0, 1}, xhat); // 屏幕右方向 = ẑ × x̂
    float u = (vdot(dW, er) / dxw) / tanH_;
    float v = (dW.z / dxw) / tanV_;
    if (par->invX) u = -u;
    if (par->invY) v = -v;
    if (par->softEdge) {
      // 软边界：中心区灵敏度不变(tanh'(0)=1)，越靠边缘每度位移越小，
      // 渐近逼近永不到达——替代"到FOV边撞墙硬停"的硬钳制
      u = tanhf(u);
      v = tanhf(v);
    }
    rawU = clampf(u, -1.0f, 1.0f);
    rawV = clampf(v, -1.0f, 1.0f);
    Xf = (rawU * 0.5f + 0.5f) * (par->scrW - 1);
    Yf = (0.5f - rawV * 0.5f) * (par->scrH - 1);
  }

  // ---- 平滑 → 死区 → 差分输出 ----
  float Xs = euroX.filter(Xf, tMs);
  float Ys = euroY.filter(Yf, tMs);
  int tx = (int)lroundf(Xs), ty = (int)lroundf(Ys);
  int dx = tx - sentX, dy = ty - sentY;
  int mag = dx < 0 ? -dx : dx; mag += dy < 0 ? -dy : dy;
  if (mag > 0 && (mag >= (int)par->deadPx || speedEma > 2.0f)) {
    if (moveCb) moveCb(dx, dy);
    sentX += dx; sentY += dy;
  }
}

void Pointer::enterFreeze() {
  frozen = true;
  haveEntry = false;
}

void Pointer::exitFreeze() {
  // 重锚定：把锚点 x̂ 旋过"冻结入口→现在"的 yaw 量，恢复运动时位置无缝衔接。
  // 守卫（防"一边有墙"）：两次水平投影都要足够长（非斜仰角），单次修正限幅 ±2.9°，
  // 超限视为噪声/真运动拒绝——否则锚点游走，有效指向窗口偏离屏幕 → 某侧转不到底。
  if (haveEntry) {
    Vec3 he = horiz(dEntry), hn = horiz(dW);
    if (vlen(he) > 0.35f && vlen(hn) > 0.35f) {
      float yaw = yawBetween(he, hn);
      if (yaw > -0.05f && yaw < 0.05f)
        xhat = rotZ(xhat, yaw);
    }
  }
  frozen = false;
  clickFrozen_ = false;
  haveEntry = false;
  driftDps = 0;
  euroX.snap(sentX); // 滤波器贴回当前输出，防恢复瞬间跳变
  euroY.snap(sentY);
}

void Pointer::recenter() {
  if (!havePrev) return; // 还没收到姿态报告时不回中
  Vec3 d = dW;
  if (vlen(d) < 0.01f && havePrev) d = qrot(lastQ, fwdVec());
  Vec3 h = horiz(d);
  if (vlen(h) > 0.1f) xhat = h; // 指向接近天顶/天底时保留旧锚点
  anchored = true;
  frozen = false;
  clickFrozen_ = false;
  haveEntry = false;
  driftDps = 0;

  uint16_t cx = (par->scrW - 1) / 2, cy = (par->scrH - 1) / 2;
  Xf = Yf = cx;
  sentX = cx; sentY = cy;
  euroX.snap(cx); euroY.snap(cy);
  if (warpCb) warpCb(cx, cy); // 用一连串相对位移把 OS 光标也拉到屏幕中心
}

// ---------------- 点击冻结 ----------------
void Pointer::clickFreeze() {
  frozen = true;        // 复用冻结通道：按下期间光标不动，点击就不会滑移
  clickFrozen_ = true;
  haveEntry = false;
}

void Pointer::clickUnfreeze() { exitFreeze(); }

bool Pointer::dragArmed() const {
  // 按住超过 freezeMs 且姿态已转超过 freezeDeg → 解除冻结转成拖拽
  if (!clickFrozen_ || !haveEntry) return false;
  return (millis() - clickDownT > par->freezeMs) &&
         (qAngleDeg(qEntry, lastQ) > par->freezeDeg);
}

// ---------------- MODE_RATE（M2：验证 HID 链路）----------------
void Pointer::onQuatRate(Quat q, uint32_t tMs) {
  lastQ = q;
  Vec3 d = qrot(q, fwdVec());
  static Vec3 prev; static bool havePrevRate = false;
  if (havePrevRate) {
    Vec3 he = horiz(prev), hn = horiz(d);
    if (vlen(he) > 0.05f && vlen(hn) > 0.05f) {
      float dyaw = yawBetween(he, hn) * RAD2DEG;
      float dpit = (asinf(clampf(d.z, -1, 1)) - asinf(clampf(prev.z, -1, 1))) * RAD2DEG;
      if (par->invX) dyaw = -dyaw;
      if (par->invY) dpit = -dpit;
      int dx = (int)lroundf(par->rateGain * dyaw);
      int dy = (int)lroundf(-par->rateGain * dpit);
      if ((dx || dy) && moveCb) moveCb(dx, dy);
    }
  }
  prev = d; havePrevRate = true;
  (void)tMs;
}
