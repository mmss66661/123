//
// Created by 20852 on 2026/1/6.
//

#ifndef HERO_GIMBAL_USB_DECODE_H
#define HERO_GIMBAL_USB_DECODE_H

#pragma once

#ifdef __cplusplus
#include "usb_protocol.h"
#include "main.h"

class USBDecode {
public:
    USBDecode();

    // 喂一个字节，返回 true 表示“解析到完整合法帧”
    bool feed(uint8_t byte);

    const USBProtocol::VisionToGimbal& packet() const;

    uint32_t errorCount() const;
    void resetErrorCount();

private:
    enum class State {
        WAIT_HEAD_0,
        WAIT_HEAD_1,
        COLLECT
    };

    State state_;
    uint8_t index_;

    uint8_t buffer_[USBProtocol::VISION_TO_GIMBAL_LEN];

    uint32_t error_count_;
};
#endif

#endif //HERO_GIMBAL_USB_DECODE_H