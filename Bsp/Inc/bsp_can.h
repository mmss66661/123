//
// Created by 20852 on 2025/9/9.
//

#ifndef STARTM3508_BSP_CAN_H
#define STARTM3508_BSP_CAN_H

#pragma once
#include "can.h"

// ===== 达妙(DM)电机模式定义 =====
// 发送帧 ID = 电机CAN_ID + 模式偏移
#define DM_MIT_MODE    0x000  // MIT模式
#define DM_POS_MODE    0x100  // 位置速度模式
#define DM_SPEED_MODE  0x200  // 速度模式

// ===== 达妙(DM)电机参数范围 (DM4340/4310等) =====
// 反馈数据解码用的物理量范围
#define DM_P_MIN  -12.5f
#define DM_P_MAX   12.5f
#define DM_V_MIN  -10.0f
#define DM_V_MAX   10.0f
#define DM_T_MIN  -28.0f
#define DM_T_MAX   28.0f

// MIT模式 KP/KD 参数范围
#define DM_KP_MIN  0.0f
#define DM_KP_MAX  500.0f
#define DM_KD_MIN  0.0f
#define DM_KD_MAX  5.0f

//大疆系列电机反馈信息
typedef struct {
    int16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度

    uint16_t last_angle;      // 上次角度
    int16_t  total_angle;     // 累计角度
    int32_t  round_count;     // 圈数计数
    uint8_t  inited;          // 是否初始化标志
} DJI_motor_info;

//达妙系列电机反馈信息
typedef struct {
    uint8_t  id;           // 电机 ID (0~15)
    // 状态/错误码：0=失能，1=使能，8~E=具体故障。
    // 不能把状态 1 当作故障，否则会造成使能、失能反复切换。
    uint8_t  err;

    uint16_t pos_raw;      // 原始位置（12bit/16bit根据型号）
    uint16_t vel_raw;      // 原始速度（12bit）
    uint16_t torque_raw;   // 原始力矩（12bit）

    uint8_t  temp_mos;     // MOS温度

    float pos;             // 物理量（可选，自动算）
    float vel;             // 物理量（可选）
    float torque;          // 物理量（可选）

    // 接收状态：用于任务层判断反馈是否在线、是否为有效的新数据。
    // 由 CAN 接收中断在完成整帧解析后更新。
    volatile uint32_t last_update_tick;
    volatile uint32_t rx_count;

} DM_motor_info;

//瓴控系列电机反馈信息
typedef struct {
    uint16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度
} LK_motor_info;

//IMU数据
typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} imu_data_info;

//底盘状态
typedef struct {
    int16_t spinvelocity;
} chassis_data;

#ifdef __cplusplus
class bsp_can {
public:
    void bsp_can_init();
    void BSP_CAN_FilterConfig();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmdNine2Eleven(int16_t motor9,int16_t motor10,int16_t motor11);

    //DM Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_DMMotorDisableCmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef BSP_CAN1_DMMotorEnableCmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef BSP_CAN1_DMMotorRefreshStatusCmd(uint16_t ID);
    HAL_StatusTypeDef BSP_CAN1_DMMotorPositionCmd(int16_t ID, float position, float velocity);
    HAL_StatusTypeDef BSP_CAN1_DMMotorVelocityCmd(int16_t ID, float velocity);
    HAL_StatusTypeDef BSP_CAN1_DMMotorMitCmd(int16_t ID, float pos, float vel, float kp, float kd, float torq);

    //LK Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_LKMotorCloseCmd(uint16_t ID);
    HAL_StatusTypeDef BSP_CAN1_LKMotorStartCmd(uint16_t ID);
    HAL_StatusTypeDef BSP_CAN1_LKMotorReadStatus2Cmd(uint16_t ID);
    HAL_StatusTypeDef BSP_CAN1_LKMotorTorqueCmd(uint16_t ID, int16_t current);
    HAL_StatusTypeDef BSP_CAN1_LKMotorVelocityCmd(uint16_t ID, int32_t velocity);
    HAL_StatusTypeDef BSP_CAN1_LKMotorIncrePosCmd(uint16_t ID, int32_t degree);
    HAL_StatusTypeDef BSP_CAN1_LKMotorIncrePosVelRestrictCmd(uint16_t ID, int32_t degree,uint32_t velocity);

    //上下半通信CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_SendRemoteControlCmd(int16_t X,int16_t Y,int16_t Z,uint8_t s2);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BSP_CAN_Init();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can2_djimotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef bsp_can2_djimotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef bsp_can2_djimotorcmdnine2eleven(int16_t motor9,int16_t motor10,int16_t motor11);

    //DM Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_dmmotordisablecmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef bsp_can1_dmmotorenablecmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef bsp_can1_dmmotorpositioncmd(int16_t ID, float position, float velocity);
    HAL_StatusTypeDef bsp_can1_dmmotorvelocitycmd(int16_t ID, float velocity);
    HAL_StatusTypeDef bsp_can1_dmmotormitcmd(int16_t ID, float pos, float vel, float kp, float kd, float torq);

    //LK motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_lkmotorclosecmd(uint16_t ID);
    HAL_StatusTypeDef bsp_can1_lkmotorstartcmd(uint16_t ID);
    HAL_StatusTypeDef bsp_can1_lkmotorreadstatus2cmd(uint16_t ID);
    HAL_StatusTypeDef bsp_can1_lkmotortorquecmd(uint16_t ID, int16_t torque);
    HAL_StatusTypeDef bsp_can1_lkmotorvelocitycmd(uint16_t ID, int32_t velocity);
    HAL_StatusTypeDef bsp_can1_lkmotorincreposcmd(uint16_t ID, int32_t degree);
    HAL_StatusTypeDef bsp_can1_lkmotorincreposvelrestrictcmd(uint16_t ID, int32_t degree,uint32_t velocity);

    //上下半通信CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_sendremotecontrolcmd(int16_t X,int16_t Y,int16_t Z,uint8_t s2);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_CAN_H
