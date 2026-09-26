//
// Created by 20852 on 2026/1/6.
//

#ifndef HERO_GIMBAL_USB_PROTOCOL_H
#define HERO_GIMBAL_USB_PROTOCOL_H

#pragma once

#ifdef __cplusplus
#include <array>
#include <cstdint>
#include <cstddef>

class USBProtocol {
public:
    //定义了自瞄模块通信协议的专属帧头与两种数据包的规定长度
    static constexpr uint8_t FRAME_HEAD_0 = 'S'; // 0x53
    static constexpr uint8_t FRAME_HEAD_1 = 'P'; // 0x50
    static constexpr size_t  VISION_TO_GIMBAL_LEN = 29;
    static constexpr size_t  GIMBAL_TO_VISION_LEN = 43;

    //三种控制模式
    enum class ControlMode : uint8_t {
        NONE         = 0,    //不控制
        CONTROL      = 1,    //控制但不开火
        CONTROL_FIRE = 2     //控制且开火
    };

    //四种云台模式
    enum class GimbalMode : uint8_t {
        FREE         = 0,    //空闲
        AIM          = 1,    //自瞄
        SMALL        = 2,    //小符
        BIG          = 3     //大符
    };

    //视觉-->云台
    struct  __attribute__((packed)) VisionToGimbal {
        uint8_t head[2];      // 'S' 'P'
        uint8_t mode;

        float yaw;
        float yaw_vel;
        float yaw_acc;

        float pitch;
        float pitch_vel;
        float pitch_acc;

        uint16_t crc16;
    };

    //云台-->视觉
    struct  __attribute__((packed)) GimbalToVision {
        uint8_t head[2];        // 'S' 'P'
        uint8_t mode;

        float q[4];

        float yaw;
        float yaw_vel;
        float pitch;
        float pitch_vel;

        float bullet_speed;
        uint16_t bullet_count;

        uint16_t crc16;
    };

    //使用“编译时断言”的C++特性用于确保两种数据包的长度严格符合设定长度，避免出现人为疏忽导致的错位的情况，在编译期规避这类错误
    static_assert(sizeof(VisionToGimbal) == VISION_TO_GIMBAL_LEN,
                  "VisionToGimbal packet size mismatch");

    static_assert(sizeof(GimbalToVision) == GIMBAL_TO_VISION_LEN,
                  "GimbalToVision size mismatch");

    //建帧函数，用于构建云台-->视觉的数据包
    static size_t buildFrame(GimbalToVision& pkt,
                               uint8_t mode,
                               std::array<float,4> q,
                               float yaw,
                               float yaw_vel,
                               float pitch,
                               float pitch_vel,
                               float bullet_speed,
                               uint16_t bullet_count);
};
#endif



#endif //HERO_GIMBAL_USB_PROTOCOL_H