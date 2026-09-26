//
// Created by 20852 on 2026/3/3.
//

#ifndef HERO_GIMBAL_FUSIONAHRS_H
#define HERO_GIMBAL_FUSIONAHRS_H

#include <array>

class GyroBiasEKF
{
public:
    GyroBiasEKF();

    void update(float gyroZ, bool isStatic);

    float getBias() const;

private:
    float x_;      // bias estimate
    float P_;      // covariance
    float Q_;      // process noise
    float R_;      // measurement noise
};

class FusionAHRS
{
public:
    FusionAHRS(float sampleFreq);

    void update(float gx, float gy, float gz,
                float ax, float ay, float az);

    std::array<float,4> getQuaternion() const;

private:
    void mahonyUpdate(float gx, float gy, float gz,
                      float ax, float ay, float az);

    bool detectStatic(float gx, float gy, float gz,
                      float ax, float ay, float az);

private:
    float sampleFreq_;

    // quaternion
    float q_[4];

    // Mahony params
    float twoKp_;
    float twoKi_;
    float integralFBx_;
    float integralFBy_;
    float integralFBz_;

    GyroBiasEKF biasEKF_;
};

#endif //HERO_GIMBAL_FUSIONAHRS_H