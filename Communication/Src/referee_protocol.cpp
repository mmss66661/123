//
// Created by 20852 on 2026/3/4.
//

#include "../Inc/referee_protocol.h"

#include <cstring>

#include "crc16.h"
#include "crc8.h"

RefereeProtocol::RefereeProtocol(FrameCallback cb)
    : buffer_(), frame_cb_(cb), data_len_(0), cmd_id_(0)
{
    memset(buffer_, 0, sizeof(buffer_));
}

void RefereeProtocol::reset()
{
    state_ = WAIT_SOF;
    index_ = 0;
    data_len_ = 0;
    cmd_id_ = 0;
}

void RefereeProtocol::input(uint8_t byte)
{
    switch(state_)
    {
    case WAIT_SOF:
        if(byte == 0xA5)
        {
            buffer_[0] = byte;
            index_ = 1;
            state_ = WAIT_LEN_L;
        }
        break;

    case WAIT_LEN_L:
        buffer_[index_++] = byte;
        data_len_ = byte;
        state_ = WAIT_LEN_H;
        break;

    case WAIT_LEN_H:
        buffer_[index_++] = byte;
        data_len_ |= (byte << 8);

        if(data_len_ > 200)  // 安全保护
        {
            reset();
        }
        else
        {
            state_ = WAIT_SEQ;
        }
        break;

    case WAIT_SEQ:
        buffer_[index_++] = byte;
        state_ = WAIT_CRC8;
        break;

    case WAIT_CRC8:
    {
        buffer_[index_++] = byte;

        uint8_t crc8 = crc8_calc(buffer_, 4,0xFF);
        if(crc8 != byte)
        {
            reset();
        }
        else
        {
            state_ = WAIT_CMDID_L;
        }
    }
        break;

    case WAIT_CMDID_L:
        buffer_[index_++] = byte;
        cmd_id_ = byte;
        state_ = WAIT_CMDID_H;
        break;

    case WAIT_CMDID_H:
        buffer_[index_++] = byte;
        cmd_id_ |= (byte << 8);

        if(data_len_ == 0)
            state_ = WAIT_CRC16_L;
        else
            state_ = WAIT_DATA;
        break;

    case WAIT_DATA:
        buffer_[index_++] = byte;

        if(index_ >= (7 + data_len_))
        {
            state_ = WAIT_CRC16_L;
        }
        break;

    case WAIT_CRC16_L:
        buffer_[index_++] = byte;
        state_ = WAIT_CRC16_H;
        break;

    case WAIT_CRC16_H:
    {
        buffer_[index_++] = byte;

        uint16_t recv_crc16 =
            buffer_[index_-2] |
            (buffer_[index_-1] << 8);

        uint16_t calc_crc16 =
            crc16_calc(buffer_, index_-2,0xFFFF);

        if(calc_crc16 == recv_crc16)
        {
            if(frame_cb_)
            {
                const uint8_t* data_ptr = &buffer_[7];
                frame_cb_(cmd_id_, data_ptr, data_len_);
            }
        }

        reset();
    }
        break;
    }
}