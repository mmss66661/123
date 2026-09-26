//
// Created by 20852 on 2026/1/6.
//

#include "../Inc/usbTask.h"

#include <array>

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "usb_decode.h"

// 协议解析器
extern USBDecode usb_decoder;

// 外部声明hUsbDeviceFS
extern USBD_HandleTypeDef hUsbDeviceFS;

//视觉到云台数据
float Vision2GimbalMode;
float Vision2GimbalYaw,Vision2GimbalYawVel,Vision2GimbalYawAcc;
float Vision2GimbalPitch,Vision2GimbalPitchVel,Vision2GimbalPitchAcc;

//云台到视觉数据
extern DJI_motor_info motor_4;//这里之后都要换成云台相关的电机
extern std::array<float,4> q;

//构建云台到视觉的反馈帧
USBProtocol::GimbalToVision pkt;

void USBTask::run() {
    uint8_t byte;

    USBProtocol::GimbalToVision tx_pkt;

    for (;;) {
        //视觉-->云台
        while (xQueueReceive(usbRxQueue, &byte, 0) == pdTRUE)
        {
            // 喂给解析器
            if (usb_decoder.feed(byte))
            {
                // 成功解析出一整帧
                const auto& pkt = usb_decoder.packet();

                //在这里使用 VisionToGimbal 数据
                Vision2GimbalMode = pkt.mode;

                Vision2GimbalYaw   = pkt.yaw;
                Vision2GimbalYawVel = pkt.yaw_vel;
                Vision2GimbalYawAcc = pkt.yaw_acc;

                Vision2GimbalPitch  = pkt.pitch;
                Vision2GimbalPitchVel = pkt.pitch_vel;
                Vision2GimbalPitchAcc = pkt.pitch_acc;
            }
        }


        //云台-->视觉
        //安全性检查
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            USBProtocol::buildFrame(
                tx_pkt,
                1,
                q,
                motor_4.rotor_angle,
                motor_4.rotor_speed,
                motor_4.rotor_angle,
                motor_4.rotor_speed,
                0,
                0
            );

            if (CDC_Transmit_FS(
                       reinterpret_cast<uint8_t*>(&tx_pkt),
                       USBProtocol::GIMBAL_TO_VISION_LEN
                   ) == USBD_BUSY) {}

        }

        //
         osDelay(10);
    }
}

extern "C" {
    static USBTask usb_task;

    void USBTask_Init() {
        usb_task.start((char*)"USBTask", 2048, osPriorityBelowNormal);
    }
}