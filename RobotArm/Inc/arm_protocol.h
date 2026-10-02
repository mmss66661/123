//
// arm_protocol.h — UART1 上位机命令通道（115200 8N1）
//
// 帧格式复用工程现有 UartProtocol：FE EF + type + len + payload + checksum
// （checksum = (type + payload 求和) & 0xFF，载荷 ≤ 27 字节，小端）。
//
// 上位机 -> 机械臂：
//   0x01 SET_ENABLE   [u8 on]            1=使能(当前位置) 0=失能
//   0x02 MOVEJ        [6*f32 关节角]     立即模式（打断当前轨迹）
//   0x03 MOVEJ_QUEUE  [6*f32 关节角]     顺序模式（排队）
//   0x04 MOVE_CART    [6*f32 x y z r p y] 末端位姿（基座系，米/弧度）
//   0x05 JOG          [u8 joint, f32 delta]
//   0x06 GRIPPER      [u8 closed]        1=闭合 0=打开
//   0x07 HEARTBEAT    []                 仅刷新链路计时（无 ACK）
//   0x08 QUERY        []                 立即回一轮 STATUS/JOINTS/POSE
//   0x0A GO_HOME      []                 自动回零：运动到标定零位(kHome 全零)
//
// 机械臂 -> 上位机：
//   0x80 ACK          [u8 cmd, u8 status] 命令回执，status 见 RequestStatus
//                                          （6=enable 执行失败，细节看 STATUS.fault）
//   0x81 STATUS       [u8 state, u8 request, u16 fault, f32 tracking_err,
//                      f32 traj_progress, u8 motor_err_mask, u8 fb_missing_mask]
//   0x82 JOINTS       [6*f32] 反馈关节角（运动学坐标）
//   0x83 POSE         [6*f32] FK 末端位姿（标定后才有）
//
// 安全：反馈超时/电机报码/CAN 失败/跟踪超时保护常开。
// （链路超时失能保护已按用户要求移除，2026-10-02。）

#ifndef ROBOT_ARM_ARM_PROTOCOL_H
#define ROBOT_ARM_ARM_PROTOCOL_H

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 由 ArmTask 调用：初始化 UartProtocol 并启动 UART1 接收
void ArmProtocol_Init(void);

// 由 ArmTask 周期调用（任务上下文）：执行命令队列、发送遥测、链路超时保护
void ArmProtocol_Tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_ARM_ARM_PROTOCOL_H
