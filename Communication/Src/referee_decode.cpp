//
// Created by 20852 on 2026/3/4.
//

#include "../Inc/referee_decode.h"

#include "referee_protocol.h"

RefereeInfo g_referee_info;
RefereeProtocol refereeproto(nullptr);
RefereeDecode referee_decode(refereeproto);

RefereeDecode::RefereeDecode(RefereeProtocol &proto) {
    // 注册回调，让协议层在解出帧后自动调用 onFrame
    proto.setFrameCallback(
        [this](uint16_t cmd_id, const uint8_t *data, uint16_t len) {
            this->onFrame(cmd_id, data, len);
        });
}


void RefereeDecode::onFrame(uint16_t cmd_id,
                            const uint8_t *data,
                            uint16_t len) {
    switch (cmd_id) {
        /***********************
         * 0x0001 比赛状态
         ***********************/
        case RefCmdID::GAME_STATUS: {
            if (len < sizeof(game_status_t))
                return;

            auto *p = reinterpret_cast<const game_status_t *>(data);

            g_referee_info.game_type = p->game_type;
            g_referee_info.game_progress = p->game_progress;
            g_referee_info.stage_remain_time = p->stage_remain_time;

            g_referee_info.updated = true;
        }
        break;

        /***********************
         * 0x0201 机器人状态
         ***********************/
        case RefCmdID::ROBOT_STATUS: {
            if (len < sizeof(robot_status_t))
                return;

            auto *p = reinterpret_cast<const robot_status_t *>(data);

            g_referee_info.robot_id = p->robot_id;
            g_referee_info.robot_level = p->robot_level;

            g_referee_info.current_hp = p->current_HP;
            g_referee_info.max_hp = p->maximum_HP;

            g_referee_info.chassis_power_limit = p->chassis_power_limit;

            g_referee_info.gimbal_output = p->power_management_gimbal_output;
            g_referee_info.chassis_output = p->power_management_chassis_output;
            g_referee_info.shooter_output = p->power_management_shooter_output;

            g_referee_info.updated = true;
        }
        break;

        /***********************
         * 0x0202 功率与热量
         ***********************/
        case RefCmdID::POWER_HEAT_DATA: {
            if (len < sizeof(power_heat_data_t))
                return;

            auto *p = reinterpret_cast<const power_heat_data_t *>(data);

            g_referee_info.buffer_energy = p->buffer_energy;

            g_referee_info.shooter_heat_17mm = p->shooter_17mm_heat;
            g_referee_info.shooter_heat_42mm = p->shooter_42mm_heat;

            g_referee_info.updated = true;
        }
        break;

        /***********************
         * 0x0203 机器人位置
         ***********************/
        case RefCmdID::ROBOT_POS: {
            if (len < sizeof(robot_pos_t))
                return;

            auto *p = reinterpret_cast<const robot_pos_t *>(data);

            g_referee_info.robot_x = p->x;
            g_referee_info.robot_y = p->y;
            g_referee_info.robot_angle = p->angle;

            g_referee_info.updated = true;
        }
        break;

        case RefCmdID::BUFF: {
            if (len < sizeof(buff_t))
                return;

            auto *p = reinterpret_cast<const buff_t *>(data);

            g_referee_info.remaining_energy = p->remaining_energy;

            g_referee_info.updated = true;
        }
        break;

        /***********************
         * 0x0206 伤害数据
         ***********************/
        case RefCmdID::HURT_DATA: {
            if (len < sizeof(hurt_data_t))
                return;

            auto *p = reinterpret_cast<const hurt_data_t *>(data);

            g_referee_info.hurt_armor_id = p->armor_id;
            g_referee_info.hurt_type = p->hurt_HP_deduction_reason;

            g_referee_info.updated = true;
        }
        break;

        /***********************
         * 0x0207 射击数据
         ***********************/
        case RefCmdID::SHOOT_DATA: {
            if (len < sizeof(shoot_data_t))
                return;

            auto *p = reinterpret_cast<const shoot_data_t *>(data);

            g_referee_info.bullet_type = p->bullet_type;
            g_referee_info.shooter_id = p->shooter_number;
            g_referee_info.shoot_freq = p->launching_frequency;
            g_referee_info.bullet_speed = p->initial_speed;

            g_referee_info.updated = true;
        }
        break;


        case RefCmdID::CUSTOM_ROBOT_DATA: // 0x0302
        {
            if (len < sizeof(custom_robot_data_t))
                return;

            auto *p = reinterpret_cast<const custom_robot_data_t *>(data);

            const uint8_t *user = p->data;

            // ===== 这里就是控制器发来的30字节 =====

            // 举例解析（你自己定义协议）
        }
        break;

        default:
            break;
    }
}

extern "C" {
void RefereeCallback(volatile uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        refereeproto.input(buf[i]);
    }
}
}
