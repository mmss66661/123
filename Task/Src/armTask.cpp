//
// armTask.cpp — 机械臂控制框架的 FreeRTOS 任务封装（含 UART1 上位机命令通道）
//
// !!! 注意 !!!
// 本任务与 MotorTask（录制/回放）、ArmDebugTask（标定调试）驱动同一组电机，
// 三者不能同时运行：在 freertos.c 中切换启动项。
//
// 上位机控制（UART1, 115200 8N1，帧格式见 RobotArm/Inc/arm_protocol.h）：
//   发 0x01 SET_ENABLE 使能后，即可用 0x02 MOVEJ / 0x04 MOVE_CART 下发目标，
//   板上自动规划轨迹并执行；遥测 STATUS/JOINTS/POSE 自动回传。
//   配套上位机工具：python tools/uart_console.py -p COMx
//   使能状态下需周期发任意帧（心跳 0x07，工具自动发），超时自动失能。
//
// 代码调用示例（本任务或任意任务中）：
//   arm::armController.enable(HAL_GetTick());
//   arm::armController.moveJ(q, HAL_GetTick());         // 关节空间（立即）
//   arm::armController.moveJQueue(q, HAL_GetTick());    // 关节空间（排队）
//   arm::armController.moveCartesian(p, HAL_GetTick()); // 末端位姿
//   arm::armController.jog(1, +0.05f, HAL_GetTick());
//   arm::armController.setGripper(true);
//   arm::armController.disable(HAL_GetTick());
//
// 调试变量：arm_fw_state / arm_fw_fault / arm_fw_request / arm_fw_ik_iters /
// arm_fw_traj_progress / arm_fw_tracking_err；协议侧 arm_proto_rx_frames 等。
//

#include "../Inc/armTask.h"

#include "arm_config.h"
#include "arm_controller.h"
#include "arm_protocol.h"

#include "main.h"

void ArmTask::run() {
    arm::armController.reset();
    ArmProtocol_Init();  // UART1 命令通道（115200，接收中断 + 命令队列）

    // 达妙上电初始化约 1 秒，留足 2 秒后再发首帧命令（沿用 motorTask 经验值）
    osDelay(arm::config::kMotorStartupDelayMs);

    for (;;) {
        const uint32_t now = HAL_GetTick();
        arm::armController.tick(now);
        ArmProtocol_Tick(now);  // 命令执行 + 遥测 + 链路超时保护
        osDelay(1);
    }
}

extern "C" {

static ArmTask armTask;

void ArmTask_Init() {
    // 栈略大于 MotorTask：IK 迭代中存在 6x6 矩阵与 FK 链的局部数组
    armTask.start((char*)"ArmTask", 2048, osPriorityNormal);
}

}
