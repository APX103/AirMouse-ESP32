#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include "airlink.h"

// 鼠标端 ESP-NOW 发送链路：广播位移/心跳/warp 包 + 统计
class Link {
public:
  bool begin();
  // 光标增量（有变化时调用，最高融合率）
  void sendDelta(int dx, int dy, uint8_t buttons);
  // 心跳：携带鼠标端绝对模型坐标（接收端据此纠偏）
  void sendHeartbeat(int absX, int absY, uint8_t buttons);
  // 回中：接收端把光标搬到 (x,y)
  void sendWarp(int x, int y);
  // 链路是否健康（最近发送无连续失败）
  bool healthy() const { return failStreak_ < 10; }

  // 统计（stat 命令/LED 用）
  uint32_t txOk = 0, txFail = 0, hbCount = 0;

private:
  bool sendPkt(const airlink_pkt_t &pkt, int len, bool withAbs);
  uint16_t seq_ = 0;
  uint8_t failStreak_ = 0;
  uint8_t peerAdded_ = false;
  int16_t lastDx_ = 0, lastDy_ = 0;           // 心跳用的累计增量
  int16_t hbAbsX_ = -1, hbAbsY_ = -1;
};
