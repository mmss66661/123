//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"

#include <sys/_intsup.h>

#include "bsp_can.h"
#include "angle_pid.h"
#include "chassis_ctrl.h"
#include "chassis_followgimbal.h"
#include "chassis_spin.h"
#include "speed_pid.h"
#include "dbus.h"
#include "debug_vars.h"
#include "tim.h"
//摩擦轮电机
extern DJI_motor_info motor_1;
extern DJI_motor_info motor_2;
extern DJI_motor_info motor_3;
//pitch轴电机
extern DJI_motor_info motor_6;
//云台yaw电机
extern LK_motor_info LK_motor_1;

//底盘控制策略结构体：初始化，模式设置，命令调用
chassis_ctrl chassis_ctrl;
//两种主动底盘控制策略：底盘跟随，小陀螺
extern chassis_followgimbal chassis_followgimbal;
extern chassis_spin chassis_spin;

//底盘控制命令结构体：XYZ三轴底盘速度
extern ChassisCmd cmd;

//云台头部板三轴状态管理
extern float yaw, pitch, roll;

//
extern imu_data_info imu_data_chassis;

//速度环PID管理
SpeedPID pitch_speed_pid(3.0f, 0.01f, 0.001f, 3500.0f, 250.0f);
SpeedPID HeroShoot_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid3(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
uint16_t pulse_width_us = 1500; // 对应90度

// 启动PWM输出

//云台pitch轴速度初始化
int16_t pitch_target_speed = 0;
//英雄三摩擦速度初始化
int16_t HeroShoot_target_speed1 = 0;
int16_t HeroShoot_target_speed2 = 0;
int16_t HeroShoot_target_speed3 = 0;


void MasterBoardTask::run() {
    pitch_speed_pid.Clear();
    HeroShoot_speed_pid1.Clear();
    HeroShoot_speed_pid2.Clear();
    HeroShoot_speed_pid3.Clear();

    for (;;) {

        //云台pitch轴速度控制
        pitch_target_speed = pitch_speed_pid.Calculate(dbus.ch[1]*1.8f, motor_6.rotor_speed, 0.001f);

        // if (pitch < -7 && pitch_target_speed > 0) {
        //     pitch_target_speed = 0 ;
        // } else if (pitch > 43 && pitch_target_speed < 0) {
        //     pitch_target_speed = 0 ;
        // }
        bsp_can2_djimotorcmdfive2eight(0,pitch_target_speed,0,0);

        // 底盘控制模式分配
        switch(dbus.s1) {
            case 1: chassis_ctrl.setMode(ChassisMode::Spin); break;
            case 3: chassis_ctrl.setMode(ChassisMode::FollowGimbal); break;
            case 2: chassis_ctrl.setMode(ChassisMode::Stop); break;
            default: break;
        }

        //发射机构及拨弹电机状态控制
        switch (dbus.s2) {
            case 1: break;
            case 2: {
                //英雄三摩擦轮速度控制（不调试发射需要注释掉）
                HeroShoot_target_speed1 = HeroShoot_speed_pid1.Calculate(-6000, motor_1.rotor_speed, 0.01f);
                HeroShoot_target_speed2 = HeroShoot_speed_pid1.Calculate(6000, motor_2.rotor_speed, 0.01f);
                HeroShoot_target_speed3 = HeroShoot_speed_pid1.Calculate(-6000, motor_3.rotor_speed, 0.01f);

                bsp_can2_djimotorcmd(HeroShoot_target_speed1,HeroShoot_target_speed2,HeroShoot_target_speed3,dbus.ch[0]*2);

                break;
            }
            case 3: {
                //英雄三摩擦轮速度控制（不调试发射需要注释掉）
                HeroShoot_target_speed1 = HeroShoot_speed_pid1.Calculate(-6000, motor_1.rotor_speed, 0.01f);
                HeroShoot_target_speed2 = HeroShoot_speed_pid1.Calculate(6000, motor_2.rotor_speed, 0.01f);
                HeroShoot_target_speed3 = HeroShoot_speed_pid1.Calculate(-6000, motor_3.rotor_speed, 0.01f);

                bsp_can2_djimotorcmd(HeroShoot_target_speed1,HeroShoot_target_speed2,HeroShoot_target_speed3,dbus.ch[0]*2);
                bsp_can1_sendremotecontrolcmd(0,0,0,dbus.s2);

                break;
            }
        }

        //云台跟随模式下的移动策略
        if(chassis_ctrl.getMode() == ChassisMode::FollowGimbal) {
            if (dbus.ch[4] != 0) {
                bsp_can1_lkmotortorquecmd(1, dbus.ch[4]*0.27f);
                chassis_followgimbal.setRCInput(dbus.ch[3]*1.5f, dbus.ch[2]*1.5f);
                chassis_followgimbal.setChassisEncoder(LK_motor_1.rotor_angle);
                chassis_followgimbal.setGimbalTarget(41687);

                // 更新策略，生成底盘命令
                chassis_ctrl.update();
            }
            else if (dbus.ch[4] == 0) {
                chassis_followgimbal.setRCInput(dbus.ch[3]*1.5f, dbus.ch[2]*1.5f);
                chassis_followgimbal.setChassisEncoder(LK_motor_1.rotor_angle);
                chassis_followgimbal.setGimbalTarget(41687);

                // 更新策略，生成底盘命令
                chassis_ctrl.update();
            }
        }
        //小陀螺模式下的移动策略
        else if(chassis_ctrl.getMode() == ChassisMode::Spin) {
            // 设置输入
            chassis_spin.setRCInput(dbus.ch[3]*1.5f, dbus.ch[2]*1.5f, 660);
            chassis_spin.setGimbalEncoder(LK_motor_1.rotor_angle);
            chassis_spin.setGimbalTarget(41687);

            // 更新策略，生成底盘命令
            chassis_ctrl.update(); // 内部调用 chassis_spin.update(cmd_)

            // 发送云台 yaw 电机命令
            bsp_can1_lkmotorvelocitycmd(1, 10100+dbus.ch[4]*9);
        }


        osDelay(10);
    }
}

extern "C" {
    static MasterBoardTask master_board_task;

    void MasterBoardTask_Init() {
        master_board_task.start((char*)"MasterBoardTask", 2048, osPriorityHigh);
    }

}
