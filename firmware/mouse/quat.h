#pragma once
#include <math.h>

#ifndef Q_PI
#define Q_PI 3.14159265358979f
#endif
#ifndef DEG2RAD
#define DEG2RAD (Q_PI / 180.0f)
#endif
#ifndef RAD2DEG
#define RAD2DEG (180.0f / Q_PI)
#endif

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

struct Vec3 {
  float x = 0, y = 0, z = 0;
};

static inline Vec3 vadd(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 vsub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 vscale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 vcross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static inline float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
static inline Vec3 vnorm(Vec3 a) {
  float l = vlen(a);
  return (l < 1e-6f) ? Vec3{} : vscale(a, 1.0f / l);
}
static inline Vec3 horiz(Vec3 v) { // 投影到水平面
  v.z = 0;
  return vnorm(v);
}

// 哈密顿四元数，约定与 SH-2 一致：v_world = q ⊗ v_body ⊗ q*
struct Quat {
  float w = 1, x = 0, y = 0, z = 0;
};

static inline Quat qmul(Quat a, Quat b) {
  Quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}
static inline Quat qconj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
static inline Quat qnorm(Quat q) {
  float l = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (l < 1e-9f) return Quat{};
  return {q.w / l, q.x / l, q.y / l, q.z / l};
}
// 旋转体坐标系向量到世界系：v' = q v q*
static inline Vec3 qrot(Quat q, Vec3 v) {
  Vec3 qv{q.x, q.y, q.z};
  Vec3 t = vscale(vcross(qv, v), 2.0f);
  return vadd(vadd(v, vscale(t, q.w)), vcross(qv, t));
}
// 两姿态间的转角（°）
static inline float qAngleDeg(Quat a, Quat b) {
  Quat d = qmul(qconj(a), b);
  float w = fabsf(d.w);
  if (w > 1.0f) w = 1.0f;
  return 2.0f * acosf(w) * RAD2DEG;
}
// 两个水平单位向量间的有符号水平角（rad，绕 +Z，b 在 a 的逆时针方向为正）
static inline float yawBetween(Vec3 a, Vec3 b) {
  float s = a.x * b.y - a.y * b.x; // (a×b).z
  float c = a.x * b.x + a.y * b.y;
  return atan2f(s, c);
}
static inline Vec3 rotZ(Vec3 v, float ang) {
  float c = cosf(ang), s = sinf(ang);
  return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}
// 水平向量的绝对方位角（rad，绕 +Z，从 +X 起）——漂移测试日志用
static inline float headingRad(Vec3 v) { return atan2f(v.y, v.x); }

// ZYX 欧拉角（deg）——仅日志显示用，不进控制链
static inline void toEulerDeg(Quat q, float &yaw, float &pitch, float &roll) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
  float sp = 2.0f * (w * y - z * x);
  pitch = asinf(clampf(sp, -1.0f, 1.0f)) * RAD2DEG;
  roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD2DEG;
}
