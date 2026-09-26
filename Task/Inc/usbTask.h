//
// Created by 20852 on 2026/1/6.
//

#ifndef HERO_GIMBAL_USBTASK_H
#define HERO_GIMBAL_USBTASK_H

#pragma once
#include "../TaskBase.h"
#include "main.h"

#ifdef __cplusplus
class USBTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    extern QueueHandle_t usbRxQueue;

    void USBTask_Init();

#ifdef __cplusplus
}
#endif


#endif //HERO_GIMBAL_USBTASK_H