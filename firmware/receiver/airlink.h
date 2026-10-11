#pragma once
#include <stdint.h>

// ============ airlink：鼠标端 ⇄ 接收端 无线协议（两份拷贝保持一致）============
#define AIRLINK_MAGIC   0xA179
#define AIRLINK_PLAIN_LEN  10                 // 无 abs 字段（2+2+2+2+1+1）
#define AIRLINK_ABS_LEN    14                 // 含 absX/absY（+2+2）
#define AIRLINK_PKT_LEN(N) ((N) + 1)          // + crc8

#define AIRLINK_FLAG_ABS  0x01                // 包含 absX/absY（同步/心跳/warp）
#define AIRLINK_FLAG_WARP 0x02                // 接收端把光标搬到 absX/absY
#define AIRLINK_BTN_L 0x01
#define AIRLINK_BTN_R 0x02

#pragma pack(push, 1)
typedef struct {
  uint16_t magic;
  uint16_t seq;
  int16_t  dx;
  int16_t  dy;
  uint8_t  buttons;
  uint8_t  flags;
  int16_t  absX;                              // flags&ABS 时有效
  int16_t  absY;
  // crc8 附在结构体末尾（打包发送时追加）
} airlink_pkt_t;
#pragma pack(pop)

// CRC-8 (poly 0x31, init 0xFF, Sensirion 风格)
static inline uint8_t airlink_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}
