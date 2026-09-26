//
// Created by 20852 on 2025/11/21.
//

#include "imuTask.h"
#include <cmath>
#include "BMI088.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "ImuTempControl.h"
#include "MahonyAHRS.h"
#include "FusionAHRS.h"
#include "debug_vars.h"

static FusionAHRS ahrs(1000.0f);   // 1kHz

float gyro[3], accel[3], temp, mag[3];
std::array<float,4> q = {1.0f, 0.0f, 0.0f, 0.0f};
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float roll_out = 0.0f, pitch_out = 0.0f, yaw_out = 0.0f;

// 上电零点
static float yaw_zero = 0.0f;
static bool first_run = true;

// 参数
#define GYRO_ZERO_THRESHOLD 0.005236f   // rad/s，对应0.3 deg/s
#define BIAS_ALPHA 0.999f          // 零偏滑动平均系数
#define LOOP_DT 0.001f             // 循环周期 1ms

void ImuTask::run() {;
    BMI088_Init();
    ImuTempControl_Init();
    DWT_Delay_ms(1000);

    // 上电时记录初始 yaw 作为零点
    BMI088_Read(gyro, accel, &temp);

    for (;;) {
        //真实数据读取
        BMI088_Read(gyro, accel, &temp);

        //控温
        ImuTempControl_Update(45, temp, 0.001f);

        // 更新融合算法
        ahrs.update(
            gyro[0],
            gyro[1],
            gyro[2],
            accel[0],
            accel[1],
            accel[2]
        );

        // 获取四元数
        q = ahrs.getQuaternion();

        //根据四元数计算欧拉角
        roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
        pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
        yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));

        //角度制转换
        roll *= (180.0f / M_PI);
        pitch *= (180.0f / M_PI);
        yaw *= (180.0f / M_PI);
        //bsp_can1_sendimudata(roll,pitch,yaw);

        if(first_run)
        {
            yaw_zero = yaw;
            first_run = false;
        }

        yaw_out = yaw - yaw_zero;
        roll_out = roll;
        pitch_out = pitch;

        debug_roll  = roll_out;
        debug_pitch = pitch_out;
        debug_yaw   = yaw_out;

        osDelay(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 1024, osPriorityAboveNormal);
    }
}