//
// Created by 20852 on 2026/3/4.
//

#ifndef HERO_GIMBAL_REFEREE_PROTOCOL_H
#define HERO_GIMBAL_REFEREE_PROTOCOL_H

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include <functional>
class RefereeProtocol
{
public:
    using FrameCallback = std::function<void(uint16_t cmd_id,const uint8_t* data,uint16_t len)>;

    explicit RefereeProtocol(FrameCallback cb);

    void setFrameCallback(FrameCallback cb){ frame_cb_ = std::move(cb); }

    void input(uint8_t byte);

private:
    enum State
    {
        WAIT_SOF,
        WAIT_LEN_L,
        WAIT_LEN_H,
        WAIT_SEQ,
        WAIT_CRC8,
        WAIT_CMDID_L,
        WAIT_CMDID_H,
        WAIT_DATA,
        WAIT_CRC16_L,
        WAIT_CRC16_H
    };

    State state_ = WAIT_SOF;

    uint8_t buffer_[256]{};
    uint16_t index_ = 0;

    uint16_t data_len_ = 0;
    uint16_t cmd_id_ = 0;

    FrameCallback frame_cb_ = nullptr;

    void reset();
};
#endif


#endif //HERO_GIMBAL_REFEREE_PROTOCOL_H