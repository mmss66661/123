//
// Created by 20852 on 2026/1/6.
//

#include "../Inc/usb_decode.h"
#include <cstring>

#include "CRC16.h"

USBDecode usb_decoder;

USBDecode::USBDecode()
    : state_(State::WAIT_HEAD_0),
      index_(0),
      error_count_(0)
{
}

bool USBDecode::feed(uint8_t byte)
{
    switch (state_) {
        case State::WAIT_HEAD_0:
            if (byte == USBProtocol::FRAME_HEAD_0) {
                buffer_[0] = byte;
                state_ = State::WAIT_HEAD_1;
            }
            break;

        case State::WAIT_HEAD_1:
            if (byte == USBProtocol::FRAME_HEAD_1) {
                buffer_[1] = byte;
                index_ = 2;
                state_ = State::COLLECT;
            } else {
                state_ = State::WAIT_HEAD_0;
                error_count_++;
            }
            break;

        case State::COLLECT:
            buffer_[index_++] = byte;

            if (index_ >= USBProtocol::VISION_TO_GIMBAL_LEN) {
                state_ = State::WAIT_HEAD_0;
                index_ = 0;

                const auto& pkt =
                    *reinterpret_cast<const USBProtocol::VisionToGimbal*>(buffer_);

                uint16_t calc = crc16_calc(
                    buffer_,
                    USBProtocol::VISION_TO_GIMBAL_LEN - 2,
                    0xFFFF
                );

                if (calc == pkt.crc16)
                {
                    error_count_ = 0;
                    return true;
                }
                else //冗余，但我在这里建议表意清晰，所以这样写
                {
                    error_count_++;
                }
            }
            break;
    }

    return false;
}

const USBProtocol::VisionToGimbal& USBDecode::packet() const
{
    return *reinterpret_cast<const USBProtocol::VisionToGimbal*>(buffer_);
}

uint32_t USBDecode::errorCount() const
{
    return error_count_;
}

void USBDecode::resetErrorCount()
{
    error_count_ = 0;
}
