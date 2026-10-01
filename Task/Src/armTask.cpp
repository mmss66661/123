//
// armTask.cpp — 机械臂控制框架的 FreeRTOS 任务封装
//
// !!! 注意 !!!
// 本任务与 MotorTask（录制/回放）驱动同一组电机，二者不能同时运行：
// 启用本任务前，请在 freertos.c 中注释掉 MotorTask_Init()。
// 框架默认上电处于 Disabled，不会主动发命令；示教/录制回放调试
// 仍可切回 MotorTask。
//
// 使用示例（在 run() 中按需取消注释）：
//
//   arm::armController.enable(HAL_GetTick());          // 使能并保持当前位置
//   arm::armController.moveJ(q, HAL_GetTick());        // 关节空间运动（立即）
//   arm::armController.moveJQueue(q, HAL_GetTick());   // 关节空间运动（排队）
//   arm::Pose p{0.25f, 0.0f, 0.30f, 0, 0, 0};
//   arm::armController.moveCartesian(p, HAL_GetTick());// 末端位姿（需先标定 DH）
//   arm::armController.jog(1, +0.05f, HAL_GetTick());  // 单关节点动
//   arm::armController.setGripper(true);               // 夹爪闭合
//   arm::armController.disable(HAL_GetTick());         // 失能/急停
//
// 调试变量（调试器观察）：arm_fw_state / arm_fw_fault / arm_fw_request /
// arm_fw_ik_iters / arm_fw_ik_err_pos / arm_fw_traj_progress / arm_fw_tracking_err
//

#include "../Inc/armTask.h"

#include "arm_controller.h"

#include "main.h"

void ArmTask::run() {
    arm::armController.reset();

    // 达妙上电初始化约 1 秒，留足 2 秒后再发首帧命令（沿用 motorTask 经验值）
    osDelay(arm::config::kMotorStartupDelayMs);

    for (;;) {
        arm::armController.tick(HAL_GetTick());
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
