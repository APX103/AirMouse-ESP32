# 📡 AirMouse-Wireless

**ESP32-S3 双板无线空中鼠标** —— [AirMouse（有线版）](../AirMouse) 的无线化：拔掉与电脑的一切连线，指哪打哪。

```
  手持端（板 B）                       接收端（板 A）
┌──────────────────────┐    ESP-NOW    ┌──────────────────┐
│ ICM20948 ─ Madgwick  │ ══(1~4ms)══►  │ 收包校验 ─ USB HID │ ══► PC
│ ray-cast + FOV+冻结  │   广播+心跳    │ 位移/按键/瞬移     │
│ 全部算法在此板       │               │ 哑桥，无算法       │
└──────────────────────┘               └──────────────────┘
```

**角色分工**：B 板 = 鼠标（接 IMU，跑全部算法）；A 板 = 接收端（一根 USB 线插电脑原生 USB 口，别的什么都不接）。

## 快速开始

```bash
# 依赖
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library"

# 编译（两端）
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M" firmware/mouse
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M" firmware/receiver

# 烧录（各自串口）
arduino-cli upload -p COM11 --fqbn esp32:esp32:esp32s3 firmware/mouse      # 板B（CH340口）
arduino-cli upload -p COM13 --fqbn esp32:esp32:esp32s3 firmware/receiver   # 板A（任一口）
```

**Arduino IDE 设置**：板型 ESP32S3 Dev Module；接收端 **USB Mode 必须选 USB-OTG (TinyUSB)**；其余默认。

## 接线

- **板 B（鼠标端）**：与有线版完全一致（IMU: VCC→3V3, GND→GND, SDA→41, SCL→42；可选按键 47/21；BOOT=回中）
- **板 A（接收端）**：无外设，USB 线插**原生 USB 口**

## 配对与使用

- 上电自动配对：接收端锁定第一个合法鼠标端（蓝灯→绿灯常亮 = 已锁定）
- 鼠标端 BOOT = 回中（光标瞬移回屏幕中心，绿灯闪）
- 静止自动冻结；接收端心跳同步防丢包漂移；断链 10s 自动解除锁定可换发射端
- 鼠标端串口（CH340 口，115200）：`stat` 看链路质量（txOk/txFail），调参与有线版同款（`help`）

## airlink 协议（12/16 字节）

`magic(0xA179) | seq | dx | dy | buttons | flags[ABS|WARP] | absX? | absY? | crc8`

位移包随动发送（≤250Hz，静止静默）；500ms 心跳携带绝对坐标供接收端纠偏（>2px 自动纠正）；
warp 包实现回中瞬移。详见 [docs/方案.md](docs/方案.md)（含选型对比、延迟预算、联调计划）。

## 延迟预算

传感器 2-4ms + I2C 1.1ms + 融合 0.02ms + **ESP-NOW 1-4ms** + HID ≤1ms ≈ **5-11ms**
（商用无线空鼠 20-60ms；有线版实测 3-6ms）

## 路线图

- [x] 协议设计与双端固件（编译通过）
- [ ] M0 双板点火与自动配对
- [ ] M1 链路质量实测（丢包率/心跳同步）
- [ ] M3 无线指哪打哪
- [ ] M4 距离/遮挡/干扰环测
- [ ] M5 电池供电 + 休眠；M6 BLE HID 直连（省掉接收端）实验

## License

MIT（同 [AirMouse](../AirMouse)）
