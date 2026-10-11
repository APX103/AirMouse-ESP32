# 📡 AirMouse-Wireless

**ESP32-S3 双板无线空中鼠标** —— [AirMouse（有线版）](../AirMouse) 的无线化：拔掉与电脑的一切连线，指哪打哪。
当前 **v0.2.6**：无限积分指针 + 硬件灵敏度旋钮。

```
  手持端（板 B）                        接收端（板 A）
┌───────────────────────┐    ESP-NOW    ┌──────────────────┐
│ ICM20948 ─ Madgwick   │ ══(1~4ms)══►  │ 收包校验 ─ USB HID │ ══► PC
│ 无限积分 + 冻结 + 旋钮 │   广播+心跳    │ 位移/按键/纠偏     │
│ 全部算法在此板        │               │ 哑桥，无算法       │
└───────────────────────┘               └──────────────────┘
```

**角色分工**：B 板 = 鼠标（接 IMU + 按键 + 灵敏度旋钮，跑全部算法）；A 板 = 接收端（一根 USB 线插电脑原生 USB 口，别的什么都不接）。

## 接线

### 板 B（鼠标端，WeAct ESP32-S3 芯片直贴版）

| IMU（微雪 ICM-20948，I2C 0x68） | 板 B | 按键 | 板 B | 灵敏度旋钮（微雪增量编码器） | 板 B |
|---|---|---|---|---|---|
| VCC | 3V3 | 左键（可选） | GPIO47 ↔ GND | VCC | 3V3 |
| GND | GND | 右键（可选） | GPIO21 ↔ GND | GND | GND |
| SDA | GPIO41 | 回中 | 板载 BOOT(GPIO0) | SIA（A相） | GPIO4 |
| SCL | GPIO42 | | | SIB（B相） | GPIO5 |
| | | | | SW（按键） | GPIO6 |

- IMU 模块自带稳压，3V3 直供即可；编码器 20 脉冲/圈，模块已带上拉，直插。
- 引脚定义集中在 [firmware/mouse/config.h](firmware/mouse/config.h)。

### 板 A（接收端）

**无外设**。USB 线插**原生 USB 口**（GPIO19/20 那个，不是 UART 口）。

## 快速开始

```bash
# 依赖
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library"

# 编译（两端同 FQBN）
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M" firmware/mouse
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M" firmware/receiver

# 烧录（各自串口；COM 号会漂移，以 arduino-cli board list 实测为准）
arduino-cli upload -p COM15 --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M,UploadSpeed=921600" firmware/mouse
arduino-cli upload -p COMxx --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,PSRAM=opi,FlashSize=16M,UploadSpeed=921600" firmware/receiver
```

**Arduino IDE 设置**：板型 ESP32S3 Dev Module；`USBMode=default` 即 USB-OTG(TinyUSB)，接收端 HID 依赖它；串口走 UART0（CDCOnBoot=default）。

⚠️ **airlink 协议一改必须双端同刷**——只刷一端 = 接收端校验不过、蓝灯永不锁定。

## 使用

- 上电自动配对：接收端锁定第一个合法鼠标端（蓝灯→绿灯常亮 = 已锁定）
- **灵敏度旋钮**：旋一格 = fovH ±2°（范围 20–150，越小越灵敏），按下 = 恢复默认 75/55，停转 5s 自动掉电保存
- 鼠标怎么动都不撞墙：无限积分模式（`mode rate`），屏幕边缘停靠交给 OS，与实体鼠标一致
- 静止自动冻结（零漂移）；接收端心跳纠偏防丢包漂移；断链 10s 自动解锁可换发射端
- 鼠标端串口（UART 口，115200，`help` 看全部命令）：`stat` 链路质量 + 芯片温度、`fov`/`mount`/`invx`/`invy` 等调参
- ⚠️ 串口打开会复位板子 → 复位后 1.5s 陀螺零偏标定，期间保持静止

## airlink 协议（10/14 字节）

`magic(0xA179) | seq | dx | dy | buttons | flags[ABS|WARP] | absX? | absY? | crc8(poly 0x31)`

位移包随动发送（≤250Hz，静止静默）；500ms 心跳携带绝对坐标供接收端纠偏（>2px 自动纠正）。
详见 [docs/方案.md](docs/方案.md)；算法原理与踩坑史见 `D:\work\note\AirMouse-算法原理与实录\index.html`。

## 延迟与温度

- 延迟：传感器 2-4ms + I2C 1.1ms + 融合 0.02ms + **ESP-NOW 1-4ms** + HID ≤1ms ≈ **5-11ms**（商用无线空鼠 20-60ms）
- 温度：CPU 240→160MHz + WiFi 发射 19.5→10dBm，芯片 ~65°C（仍偏高，见路线图 BLE/星闪实验）

## 路线图

- [x] 协议设计与双端固件
- [x] M0 双板点火与自动配对；M1 链路实测 263 包/s 零失败
- [x] M3 无线指哪打哪（v0.2.3 起无限积分模式，实测无空气墙）
- [x] v0.2.6 灵敏度旋钮（增量编码器，实测无跳格）
- [ ] M4 距离/遮挡/干扰环测
- [ ] M5 电池供电 + 休眠
- [ ] M6 更凉快的链路实验：BLE HID 直连 / 星闪(SLE) 移植（换板）

## License

MIT（同 [AirMouse](../AirMouse)）
