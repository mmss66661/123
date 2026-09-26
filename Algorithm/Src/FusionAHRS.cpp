//
// Created by 20852 on 2026/3/3.
//

#include "../Inc/FusionAHRS.h"

#include <cmath>

#define GRAVITY 9.81f
#define GYRO_STATIC_THRESH 0.02f
#define ACC_STATIC_THRESH  0.2f

/* ===========================
   GyroBiasEKF
   =========================== */

GyroBiasEKF::GyroBiasEKF()
{
    x_ = 0.0f;
    P_ = 0.1f;
    Q_ = 1e-6f;
    R_ = 1e-4f;
}

void GyroBiasEKF::update(float gyroZ, bool isStatic)
{
    // 预测
    P_ += Q_;

    if(isStatic)
    {
        float K = P_ / (P_ + R_);
        x_ += K * (gyroZ - x_);
        P_ = (1 - K) * P_;
    }
}

float GyroBiasEKF::getBias() const
{
    return x_;
}


FusionAHRS::FusionAHRS(float sampleFreq)
    : sampleFreq_(sampleFreq),
      twoKp_(1.0f),
      twoKi_(0.0f),
      integralFBx_(0),
      integralFBy_(0),
      integralFBz_(0)
{
    q_[0] = 1.0f;
    q_[1] = 0.0f;
    q_[2] = 0.0f;
    q_[3] = 0.0f;
}

bool FusionAHRS::detectStatic(float gx, float gy, float gz,
                              float ax, float ay, float az)
{
    float gyroNorm = std::sqrt(gx*gx + gy*gy + gz*gz);
    float accNorm  = std::sqrt(ax*ax + ay*ay + az*az);

    return (gyroNorm < GYRO_STATIC_THRESH &&
            std::fabs(accNorm - GRAVITY) < ACC_STATIC_THRESH);
}

void FusionAHRS::update(float gx, float gy, float gz,
                        float ax, float ay, float az)
{
    bool isStatic = detectStatic(gx, gy, gz, ax, ay, az);

    // 更新 bias EKF
    biasEKF_.update(gz, isStatic);

    // 去偏
    gz -= biasEKF_.getBias();

    // 进入 Mahony
    mahonyUpdate(gx, gy, gz, ax, ay, az);
}

void FusionAHRS::mahonyUpdate(float gx, float gy, float gz,
                              float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!(ax == 0 && ay == 0 && az == 0))
    {
        recipNorm = 1.0f / std::sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q_[1]*q_[3] - q_[0]*q_[2];
        halfvy = q_[0]*q_[1] + q_[2]*q_[3];
        halfvz = q_[0]*q_[0] - 0.5f + q_[3]*q_[3];

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        gx += twoKp_ * halfex;
        gy += twoKp_ * halfey;
        gz += twoKp_ * halfez;
    }

    gx *= (0.5f / sampleFreq_);
    gy *= (0.5f / sampleFreq_);
    gz *= (0.5f / sampleFreq_);

    qa = q_[0];
    qb = q_[1];
    qc = q_[2];

    q_[0] += (-qb*gx - qc*gy - q_[3]*gz);
    q_[1] += (qa*gx + qc*gz - q_[3]*gy);
    q_[2] += (qa*gy - qb*gz + q_[3]*gx);
    q_[3] += (qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / std::sqrt(q_[0]*q_[0] + q_[1]*q_[1] +
                                 q_[2]*q_[2] + q_[3]*q_[3]);

    q_[0] *= recipNorm;
    q_[1] *= recipNorm;
    q_[2] *= recipNorm;
    q_[3] *= recipNorm;
}

std::array<float,4> FusionAHRS::getQuaternion() const
{
    return {q_[0], q_[1], q_[2], q_[3]};
}