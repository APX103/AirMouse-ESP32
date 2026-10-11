/*
 * AirMouse-Wireless 接收端（板 A → 电脑）
 * 职责：ESP-NOW 收包（airlink）→ 校验 → USB HID 鼠标
 * 自动配对：锁定第一个合法源 MAC（magic+crc 通过），断链10秒后允许重锁
 *
 * IDE：板型 ESP32S3 Dev Module
 *   USB Mode: USB-OTG (TinyUSB)   ← 必须（HID）
 *   USB CDC On Boot: Disabled      ← Serial=UART0（CH340口）出状态日志
 * 接线：什么都不接。USB 线插【原生 USB 口】(GPIO19/20)。
 */
#include <USB.h>
#include <USBHIDMouse.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp32-hal-rgb-led.h"
#include "airlink.h"

#define PIN_LED 48

USBHIDMouse Mouse;

// ---- 链路状态 ----
static uint8_t lockMac[6];
static bool locked = false;
static uint32_t lastPktMs = 0;
static uint16_t lastSeq = 0;
static bool haveSeq = false;

// ---- 接收端模型（与鼠标端心跳同步）----
static int modelX = 960, modelY = 540;
static uint8_t btnState = 0;

// ---- 统计 ----
static uint32_t rxOk = 0, rxBad = 0, rxDup = 0, rxOther = 0, warpCount = 0, resyncCount = 0;

static void applyMove(int dx, int dy) {
  if (dx < -120) dx = -120; if (dx > 120) dx = 120;
  if (dy < -120) dy = -120; if (dy > 120) dy = 120;
  if (dx || dy) Mouse.move((int8_t)dx, (int8_t)dy);
  modelX += dx; modelY += dy;
}

static void warpTo(int x, int y) {  // 分块搬运（视觉瞬移）
  while (modelX < x) { int s = (x - modelX > 100) ? 100 : (x - modelX); Mouse.move((int8_t)s, 0); modelX += s; }
  while (modelX > x) { int s = (modelX - x > 100) ? 100 : (modelX - x); Mouse.move((int8_t)-s, 0); modelX -= s; }
  while (modelY < y) { int s = (y - modelY > 100) ? 100 : (y - modelY); Mouse.move(0, (int8_t)s); modelY += s; }
  while (modelY > y) { int s = (modelY - y > 100) ? 100 : (modelY - y); Mouse.move(0, (int8_t)-s); modelY -= s; }
}

static void applyButtons(uint8_t b) {
  if (b == btnState) return;
  if ((b & AIRLINK_BTN_L) != (btnState & AIRLINK_BTN_L)) {
    (b & AIRLINK_BTN_L) ? Mouse.press(MOUSE_LEFT) : Mouse.release(MOUSE_LEFT);
  }
  if ((b & AIRLINK_BTN_R) != (btnState & AIRLINK_BTN_R)) {
    (b & AIRLINK_BTN_R) ? Mouse.press(MOUSE_RIGHT) : Mouse.release(MOUSE_RIGHT);
  }
  btnState = b;
}

// ESP-NOW 接收回调（IDF 5.x 签名）
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  uint32_t now = millis();
  bool absPkt = (len == AIRLINK_ABS_LEN + 1);
  if (!absPkt && len != AIRLINK_PLAIN_LEN + 1) { rxOther++; return; }
  int body = len - 1;
  if (data[0] != (AIRLINK_MAGIC & 0xFF) || data[1] != (AIRLINK_MAGIC >> 8)) { rxBad++; return; }
  if (airlink_crc8(data, body) != data[body]) { rxBad++; return; }

  const uint8_t *src = info->src_addr;
  if (!locked) {
    memcpy(lockMac, src, 6);
    locked = true; haveSeq = false;
    Serial.print("[link] 锁定鼠标端 MAC ");
    for (int i = 0; i < 6; i++) Serial.printf("%02X%s", lockMac[i], i < 5 ? ":" : "\n");
  } else if (memcmp(src, lockMac, 6) != 0) {
    rxOther++;  // 其他 ESP-NOW 设备，忽略
    return;
  }

  airlink_pkt_t pkt;
  memcpy(&pkt, data, body);
  if (haveSeq && pkt.seq == lastSeq) { rxDup++; return; }
  lastSeq = pkt.seq; haveSeq = true;
  lastPktMs = now;
  rxOk++;

  applyButtons(pkt.buttons);
  if (pkt.flags & AIRLINK_FLAG_WARP) {
    warpTo(pkt.absX, pkt.absY);
    warpCount++;
  } else {
    applyMove(pkt.dx, pkt.dy);
    if (pkt.flags & AIRLINK_FLAG_ABS) {           // 心跳：纠偏
      int ex = pkt.absX - modelX, ey = pkt.absY - modelY;
      if (ex > 2 || ex < -2 || ey > 2 || ey < -2) {
        applyMove(ex, ey);
        resyncCount++;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  neopixelWrite(PIN_LED, 8, 0, 8);   // 紫：等待配对
  Serial.println("\n=== AirMouse-WL 接收端 (板A → USB HID) ===");

  Mouse.begin();
  USB.begin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false);
  // 强制驻留信道1（与鼠标端一致；默认信道受 NVS 历史影响可能各说各话）
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  uint8_t myMac[6];
  esp_wifi_get_mac(WIFI_IF_STA, myMac);  // 驱动初始化后即可读（macAddress()要start后才有效）
  Serial.printf("本机 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  Serial.println("等待鼠标端广播…（先开板B，或按板B复位）");
  if (esp_now_init() != ESP_OK) {
    Serial.println("!! esp_now_init 失败");
    return;
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  uint32_t now = millis();

  // 断链 10 秒解除锁定（允许换鼠标端）
  if (locked && now - lastPktMs > 10000) {
    locked = false;
    Serial.println("[link] 断链超时，解除锁定，等待重新配对");
  }

  // 状态灯：蓝=等配对 绿=已锁定常亮 红闪=曾锁定但丢链(1~10s)
  if (!locked)            neopixelWrite(PIN_LED, 0, 0, 12);
  else if (now - lastPktMs < 1000) neopixelWrite(PIN_LED, 0, 14, 0);
  else                    neopixelWrite(PIN_LED, (now / 300) & 1 ? 20 : 0, 0, 0);

  // 串口状态（每 2 秒）
  static uint32_t tStat = 0;
  if (now - tStat >= 2000) {
    tStat = now;
    Serial.printf("[stat] ok=%lu bad=%lu dup=%lu other=%lu warp=%lu resync=%lu model=(%d,%d) %s\n",
                  (unsigned long)rxOk, (unsigned long)rxBad, (unsigned long)rxDup,
                  (unsigned long)rxOther, (unsigned long)warpCount,
                  (unsigned long)resyncCount, modelX, modelY,
                  locked ? "LINKED" : "WAIT");
  }
}
