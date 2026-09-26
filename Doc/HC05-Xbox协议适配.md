# HC-05 / Xbox 手柄协议适配

## 硬件连接

USART1 使用接收模式、9600 baud、8N1：

| HC-05 | STM32F407IGH6 |
|---|---|
| TXD | PB7 / USART1_RX |
| GND | GND |

本工程的 PA9、PA10、PB6 已分别被 TIM1_CH2、USB_OTG_FS_ID、CAN2_TX 占用，
因此 USART1 只能使用空闲的 PB7 做接收。MCU 不向 HC-05 发送数据，HC-05 的 RXD
可以不接。若模块数据模式不是 9600 baud，需要同步修改 `MX_USART1_UART_Init()`。

## 20 字节帧

接收格式与 `mmss66661/XBOX_HC05` 仓库一致：

| 偏移 | 字段 | 格式 |
|---:|---|---|
| 0..1 | 帧头 | `AA 55` |
| 2 | 协议版本 | `01` |
| 3 | 序号 | `uint8_t`，循环递增 |
| 4..5 | 按键 | `uint16_t`，小端 |
| 6..13 | LX/LY/RX/RY | 四个 `int16_t`，小端 |
| 14..15 | LT/RT | 两个 `uint8_t` |
| 16..17 | HAT X/Y | 两个 `int8_t`，范围 -1..1 |
| 18..19 | CRC | CRC16/MODBUS，小端，校验字节 2..17 |

USART1 中断逐字节调用 `HC05_Gamepad_FeedByte()`。解析器会寻找帧头、校验版本和
CRC，并在串流错位或坏帧后自动重新同步。

## 当前控制映射

为复用现有机械臂任务，完整 Xbox 状态解析后同步到原 `dbus` 控制结构：

| Xbox 输入 | 现有控制量 | 用途 |
|---|---|---|
| LX | `dbus.ch[0]` | J0 |
| LY（取反） | `dbus.ch[1]` | J1 |
| RX | `dbus.ch[2]` | J2 |
| RY（取反） | `dbus.ch[3]` | J3 |
| RT - LT | `dbus.ch[4]` | J4/J5 差速腕部 |
| RB（按住） | 安全使能 | 松开后立即进入安全失能流程 |
| RB + A | `dbus.s1 = 1` | 夹爪闭合 |
| RB + B | `dbus.s1 = 3` | 夹爪打开 |
| RB + BACK（按住） | `dbus.s2 = 1` | 录制 |
| RB，无模式键 | `dbus.s2 = 2` | 手动 |
| RB + START（按住） | `dbus.s2 = 3` | 回放 |

必须始终按住 RB 才接受控制，避免 App 在手柄未就绪时持续发送全零帧造成误使能。
松开 A/B 后夹爪保持上一次状态；BACK 与 START 同时按下按失能处理。回放挡仍需
连续按住 RB + START 约 600 ms，沿用原有防误触逻辑。

## 调试变量

可在调试器中查看全局变量 `hc05_gamepad`：

- `frame_count`：通过全部校验的帧数；
- `last_update_tick`：最近有效帧时间；
- `dropped_frame_count`：按序号估算的丢帧数；
- `crc_error_count`：CRC 错误数；
- `format_error_count`：版本、范围或保留位错误数；
- `uart_error_count`：USART1 的 ORE/NE/FE/PE 错误数；
- 其余字段为最近一帧的原始 Xbox 状态。

现有 `MotorTask` 的 100 ms 遥控器超时保护保持不变；HC-05 停止发送后电机会按原
逻辑失能，恢复后先处于手动挡才允许重新使能。
