//
// Created by 20852 on 2026/1/6.
//

#include "../Inc/usb_protocol.h"

#include <array>

#include "CRC16.h"

size_t USBProtocol::buildFrame(GimbalToVision& pkt,
                               uint8_t mode,
                               const std::array<float,4> q,
                               float yaw,
                               float yaw_vel,
                               float pitch,
                               float pitch_vel,
                               float bullet_speed,
                               uint16_t bullet_count)
{
    pkt.head[0] = FRAME_HEAD_0;
    pkt.head[1] = FRAME_HEAD_1;

    pkt.mode = mode;

    // 四元数
    for (int i = 0; i < 4; ++i)
        pkt.q[i] = q[i];

    pkt.yaw       = yaw;
    pkt.yaw_vel   = yaw_vel;
    pkt.pitch     = pitch;
    pkt.pitch_vel = pitch_vel;

    pkt.bullet_speed = bullet_speed;
    pkt.bullet_count = bullet_count;

    pkt.crc16 = 0;   // 关键清零

    // CRC 计算（除去最后2字节）
    uint16_t crc = crc16_calc(
        reinterpret_cast<const uint8_t*>(&pkt),
        GIMBAL_TO_VISION_LEN - 2,
        0xFFFF
    );

    pkt.crc16 = crc;

    return GIMBAL_TO_VISION_LEN;
}