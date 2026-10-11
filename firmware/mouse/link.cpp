#include "link.h"
#include <esp_wifi.h>   // esp_wifi_set_max_tx_power

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool Link::begin() {
  WiFi.mode(WIFI_STA);            // 不关联任何 AP
  WiFi.disconnect(false);
  // 强制驻留信道1：未关联 STA 的"默认信道"受 NVS 里历史 WiFi 配置影响，
  // 两板可能各在各的信道 → 谁也收不到谁（ESP-NOW 经典坑），钉死才稳
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;               // 0 = 当前信道
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  peerAdded_ = true;
  // 降温&省电：发射功率 19.5dBm → 10dBm（室内距离足够）
  esp_wifi_set_max_tx_power(40);  // 单位 0.25dBm
  return true;
}

bool Link::sendPkt(const airlink_pkt_t &pkt, int len, bool withAbs) {
  if (!peerAdded_) return false;
  uint8_t buf[AIRLINK_ABS_LEN + 1];
  int body = withAbs ? AIRLINK_ABS_LEN : AIRLINK_PLAIN_LEN;
  memcpy(buf, &pkt, body);
  buf[body] = airlink_crc8(buf, body);
  seq_++;
  esp_err_t r = esp_now_send(BROADCAST_MAC, buf, body + 1);
  if (r == ESP_OK) { txOk++; failStreak_ = 0; return true; }
  txFail++; if (failStreak_ < 255) failStreak_++;
  return false;
}

void Link::sendDelta(int dx, int dy, uint8_t buttons) {
  airlink_pkt_t p = {};
  p.magic = AIRLINK_MAGIC;
  p.seq = seq_;
  p.dx = (int16_t)dx;
  p.dy = (int16_t)dy;
  p.buttons = buttons;
  p.flags = 0;
  sendPkt(p, AIRLINK_PLAIN_LEN, false);
}

void Link::sendHeartbeat(int absX, int absY, uint8_t buttons) {
  airlink_pkt_t p = {};
  p.magic = AIRLINK_MAGIC;
  p.seq = seq_;
  p.dx = 0;
  p.dy = 0;
  p.buttons = buttons;
  p.flags = AIRLINK_FLAG_ABS;
  p.absX = (int16_t)absX;
  p.absY = (int16_t)absY;
  sendPkt(p, AIRLINK_ABS_LEN, true);
  hbCount++;
}

void Link::sendWarp(int x, int y) {
  airlink_pkt_t p = {};
  p.magic = AIRLINK_MAGIC;
  p.seq = seq_;
  p.dx = 0;
  p.dy = 0;
  p.buttons = 0;
  p.flags = AIRLINK_FLAG_ABS | AIRLINK_FLAG_WARP;
  p.absX = (int16_t)x;
  p.absY = (int16_t)y;
  sendPkt(p, AIRLINK_ABS_LEN, true);
}
