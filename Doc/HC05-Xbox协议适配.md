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
| RB（单击） | `dbus.s2 = 1` | 开始录制并锁存 |
| LB（单击） | `dbus.s2 = 3` | 开始回放并锁存 |
| START（单击） | `dbus.s2 = 0` | 立即停止、失能电机并擦除录制区 |
| A | `dbus.s1 = 1` | 夹爪闭合 |
| B | `dbus.s1 = 3` | 夹爪打开 |

工作模式在按钮按下沿切换，按钮松开后继续保持，不需要长按。START 的优先级最高；
发生故障后可直接再次单击 RB 或 LB 重试。BACK 不参与任何控制。
松开 A/B 后夹爪保持上一次状态。

START 每次按下都会擦除 Flash sector 11 中的全部录制数据；即使当前已经处于停止状态，
再次单击也会执行擦除。因此要回放刚录制的轨迹，应在录制过程中直接单击 LB，不能先按
START。内部 Flash 有擦写寿命限制，不要高频反复点击 START。

## 调试变量

可在调试器中查看全局变量 `hc05_gamepad`：

- `frame_count`：通过全部校验的帧数；
- `last_update_tick`：最近有效帧时间；
- `dropped_frame_count`：按序号估算的丢帧数；
- `crc_error_count`：CRC 错误数；
- `format_error_count`：版本、范围或保留位错误数；
- `uart_error_count`：USART1 的 ORE/NE/FE/PE 错误数；
- `operation_mode`：`0` 停止、`1` 录制、`3` 回放；
- 其余字段为最近一帧的原始 Xbox 状态。

现有 `MotorTask` 的 100 ms 遥控器超时保护保持不变；HC-05 停止发送后电机会按原
逻辑失能并自动回到停止状态，恢复通信后需要重新单击 RB 或 LB。

## RGB 工作状态灯

板载 RGB LED 为高电平点亮：PH10 蓝、PH11 绿、PH12 红。当前状态定义为：

- 停止：绿灯亮；
- 确实进入录制：红灯与绿灯同时亮，显示黄色；
- 确实进入返回起点/回放：红灯亮；
- 反馈超时或 CAN 发送失败：蓝灯；
- 电机自身报码：紫灯（红 + 蓝）；
- Flash 擦除或写入失败：青灯（绿 + 蓝）；
- 无有效记录或位置超限：白灯（红 + 绿 + 蓝）；
- 超过 100 ms 没有正确帧：立即停止并恢复绿灯；

灯色由电机任务的真实状态驱动，不再仅表示手柄按钮已经收到。

当前 USART1 为 9600 baud、8N1，每个 UART 字节实际占 10 bit。20 字节协议帧以
100 Hz 发送需要约 20 kbit/s，超过 9600 baud；50 Hz 也需要约 10 kbit/s。保持
9600 baud 时建议上位机设为 20 Hz。若需要 100 Hz，应先把 HC-05 数据串口和
`MX_USART1_UART_Init()` 同时改为 115200 baud。

## 电机不能使能时的诊断

录制和回放启动前仍要求 J0~J5 都能返回新的 CAN 反馈。进入故障后可直接再次单击
RB 或 LB，系统会把它视为新的启动请求；START 则停止并清空记录。

可在调试器中观察：

- `arm_remote_raw_mode`：原始锁存模式，`0` 停止、`1` 录制、`3` 回放；
- `arm_requested_mode`：滤波后的模式，按键后约 50 ms 应变为对应值；
- `arm_control_state`：`0` 停止、`2` 录制、`4/5/6` 回放流程、`7` 故障；
- `arm_fault_code`：`1` 为手柄超时，`2` 为电机反馈超时，`3` 为电机报码，
  `7` 为 CAN 发送失败；
- `arm_missing_feedback_mask`：缺失反馈位图，bit0~bit5 分别对应 J0~J5；
- `arm_motor_error_mask`：电机报码位图，bit0~bit5 分别对应 J0~J5；
- `arm_can_tx_failed_joint`：CAN 发送失败的关节序号，`-1` 表示没有发送失败。
- `DM_feedback_master_id[0..5]`：J0~J5 实际收到反馈时使用的 Master ID。

例如 `arm_missing_feedback_mask == 0x20` 表示只有 J5 未反馈，`0x3F` 表示六台均
未反馈，应重点检查 CAN1 接线、1 Mbps 波特率、终端电阻，以及电机实际 CAN ID
是否与代码中的 `0x02~0x07` 一致。反馈仲裁 ID 使用电机独立配置的 Master ID，固件
现根据反馈数据中的电机 ID 自动识别，不再写死为 `0x12~0x17`。反馈握手按关节逐台等待，
优先使用官方 `0x7FF/0xCC` 状态刷新命令；对不支持该命令的旧固件回退到原失能命令。
每种方式每台最多重试 3 次、每次 20 ms，避免原先统一等待 10 ms 导致误判。
