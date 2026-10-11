#pragma once
#include "quat.h"

// 6 轴 Madgwick AHRS（陀螺+加计）。输出约定与 BNO085 Game Rotation Vector 同构：
// q 满足 qrot(q, v_body) = v_world，世界系 Z 轴向上（重力对齐）→ pitch/roll 不漂，yaw 慢漂
struct Madgwick6 {
  Quat q;
  float beta = 0.12f;
  uint32_t tPrevMs = 0;
  bool inited = false;

  static Quat alignAToB(Vec3 a, Vec3 b) { // 最短弧旋转：把单位向量 a 转到 b
    float d = clampf(vdot(a, b), -1.0f, 1.0f);
    Vec3 c = vcross(a, b);
    float s = sqrtf(0.5f * (1.0f + d));
    if (s < 1e-4f) { // 反向：绕任意垂直轴 180°
      Vec3 ax = fabsf(a.y) < 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
      Vec3 n = vnorm(vcross(a, ax));
      return qnorm(Quat{0, n.x, n.y, n.z});
    }
    float inv = 0.5f / s;
    return qnorm(Quat{s, c.x * inv, c.y * inv, c.z * inv});
  }

  // gx..gz: rad/s（零偏已扣除）；ax..az: g；tMs: millis
  Quat update(float gx, float gy, float gz, float ax, float ay, float az, uint32_t tMs) {
    float al = sqrtf(ax * ax + ay * ay + az * az);
    if (!inited) {
      if (al > 0.5f && al < 1.5f) { // 重力向量有效 → 初始姿态（yaw 随机，无所谓）
        Vec3 up = vnorm(Vec3{ax, ay, az});
        q = alignAToB(up, Vec3{0, 0, 1});
        tPrevMs = tMs;
        inited = true;
      }
      return q;
    }
    float dt = (tMs - tPrevMs) / 1000.0f;
    tPrevMs = tMs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    float w = q.w, x = q.x, y = q.y, z = q.z;

    // 陀螺积分项：qDot = 0.5 * q ⊗ ω
    float dqw = 0.5f * (-x * gx - y * gy - z * gz);
    float dqx = 0.5f * (w * gx + y * gz - z * gy);
    float dqy = 0.5f * (w * gy - x * gz + z * gx);
    float dqz = 0.5f * (w * gz + x * gy - y * gx);

    // 加计梯度修正：h = 预测的机体系重力方向；e = h - a
    if (al > 0.3f) {
      float inv = 1.0f / al;
      ax *= inv; ay *= inv; az *= inv;
      float hx = 2.0f * (z * x - w * y);
      float hy = 2.0f * (w * x + z * y);
      float hz = 1.0f - 2.0f * (x * x + y * y);
      float ex = hx - ax, ey = hy - ay, ez = hz - az;
      // 梯度 Jᵀe
      float gwv = -2 * y * ex + 2 * x * ey;
      float gxv = 2 * z * ex + 2 * w * ey - 4 * x * ez;
      float gyv = -2 * w * ex + 2 * z * ey - 4 * y * ez;
      float gzv = 2 * x * ex + 2 * y * ey;
      float gn = sqrtf(gwv * gwv + gxv * gxv + gyv * gyv + gzv * gzv);
      if (gn > 1e-6f) {
        float s = beta / gn;
        dqw -= gwv * s; dqx -= gxv * s; dqy -= gyv * s; dqz -= gzv * s;
      }
    }

    q.w += dqw * dt; q.x += dqx * dt; q.y += dqy * dt; q.z += dqz * dt;
    q = qnorm(q);
    return q;
  }
};
