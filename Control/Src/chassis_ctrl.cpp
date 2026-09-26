//
// Created by 20852 on 2025/12/29.
//
#include "../Inc/chassis_ctrl.h"

#include "bsp_can.h"

#include "chassis_ctrl.h"
#include "chassis_strategy.h"

#include "chassis_stop.h"
#include "chassis_followgimbal.h"
#include "chassis_spin.h"

//静态对象 用于切换操作模式
chassis_stop         chassis_stop;
chassis_followgimbal chassis_followgimbal;
chassis_spin         chassis_spin;

//初始化底盘控制状态为静止模式
void chassis_ctrl::init()
{
    setMode(ChassisMode::Stop);
}

//设置当前运动模式
void chassis_ctrl::setMode(ChassisMode mode)
{
    mode_ = mode;

    switch (mode_)
    {
        case ChassisMode::FollowGimbal:
            strategy_ = &chassis_followgimbal;
            break;

        case ChassisMode::Spin:
            strategy_ = &chassis_spin;
            break;

        case ChassisMode::Stop:
        default:
            strategy_ = &chassis_stop;
            break;
    }
}

ChassisMode chassis_ctrl::getMode() const
{
    return mode_;
}

void chassis_ctrl::update()
{
    //清空指令
    cmd_ = {};

    if (strategy_)
    {
        strategy_->update(cmd_);
    }

    //发送底盘命令
    bsp_can1_sendremotecontrolcmd(cmd_.vx, cmd_.vy, -cmd_.wz,dbus.s2);

    if (mode_  == ChassisMode::FollowGimbal && dbus.ch[4] == 0) {
        bsp_can1_lkmotortorquecmd(1, cmd_.wz * 0.35f);
    }
}
