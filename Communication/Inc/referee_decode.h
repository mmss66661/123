//
// Created by 20852 on 2026/3/4.
//

#ifndef HERO_GIMBAL_REFEREE_DECODE_H
#define HERO_GIMBAL_REFEREE_DECODE_H

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include "referee_protocol.h"

#pragma pack(push,1)

//裁判系统帧头
struct referee_frame_header_t
{
    uint8_t  sof;          // 数据帧起始字节 0xA5
    uint16_t data_length;  // data长度
    uint8_t  seq;          // 包序号
    uint8_t  crc8;         // 对上面三者进行CRC8校验
};

//命令码ID定义：
namespace RefCmdID
{
    /************** 比赛信息 **************/
    constexpr uint16_t GAME_STATUS       = 0x0001;
    constexpr uint16_t GAME_RESULT       = 0x0002;
    constexpr uint16_t ROBOT_HP          = 0x0003;

    /************** 场地事件 **************/
    constexpr uint16_t EVENT_DATA        = 0x0101;
    constexpr uint16_t REFEREE_WARNING   = 0x0104;
    constexpr uint16_t DART_INFO         = 0x0105;

    /************** 机器人信息 **************/
    constexpr uint16_t ROBOT_STATUS      = 0x0201;
    constexpr uint16_t POWER_HEAT_DATA   = 0x0202;
    constexpr uint16_t ROBOT_POS         = 0x0203;
    constexpr uint16_t BUFF              = 0x0204;

    constexpr uint16_t HURT_DATA         = 0x0206;
    constexpr uint16_t SHOOT_DATA        = 0x0207;

    constexpr uint16_t PROJECTILE_ALLOW  = 0x0208;
    constexpr uint16_t RFID_STATUS       = 0x0209;

    constexpr uint16_t DART_CLIENT_CMD   = 0x020A;
    constexpr uint16_t GROUND_ROBOT_POS  = 0x020B;
    constexpr uint16_t RADAR_MARK_DATA   = 0x020C;
    constexpr uint16_t SENTRY_INFO       = 0x020D;
    constexpr uint16_t RADAR_INFO        = 0x020E;

    /************** 机器人交互 **************/
    constexpr uint16_t ROBOT_INTERACTION = 0x0301;
    constexpr uint16_t CUSTOM_ROBOT_DATA = 0x0302;
    constexpr uint16_t MAP_COMMAND       = 0x0303;
    constexpr uint16_t REMOTE_CONTROL    = 0x0304;
    constexpr uint16_t MAP_ROBOT_DATA    = 0x0305;
    constexpr uint16_t CUSTOM_CLIENT_DATA= 0x0306;
    constexpr uint16_t MAP_DATA          = 0x0307;
    constexpr uint16_t CUSTOM_INFO       = 0x0308;
    constexpr uint16_t ROBOT_CUSTOM_DATA = 0x0309;
    constexpr uint16_t ROBOT_CUSTOM_DATA_2 = 0x0310;

    /************** 雷达/图传 **************/
    constexpr uint16_t RADAR_CHANNEL_SET   = 0x0F01;
    constexpr uint16_t RADAR_CHANNEL_QUERY = 0x0F02;

    /************** 敌方信息 **************/
    constexpr uint16_t ENEMY_ROBOT_POS        = 0x0A01;
    constexpr uint16_t ENEMY_ROBOT_HP         = 0x0A02;
    constexpr uint16_t ENEMY_PROJECTILE_ALLOW = 0x0A03;
    constexpr uint16_t ENEMY_MACRO_STATUS     = 0x0A04;
    constexpr uint16_t ENEMY_BUFF_STATUS      = 0x0A05;
    constexpr uint16_t INTERFERENCE_KEY       = 0x0A06;
}

/***********************
 * 0x0001 比赛状态数据
 ***********************/
struct game_status_t
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;

 uint16_t stage_remain_time;

 uint64_t SyncTimeStamp;
};

/***********************
 * 0x0002 比赛结果数据
 ***********************/
struct game_result_t
{
 uint8_t winner;
};

/***********************
 * 0x0002 机器人血量数据
 ***********************/
struct game_robot_HP_t
{
 uint16_t ally_1_robot_HP;
 uint16_t ally_2_robot_HP;
 uint16_t ally_3_robot_HP;
 uint16_t ally_4_robot_HP;

 uint16_t reserved;

 uint16_t ally_7_robot_HP;
 uint16_t ally_outpost_HP;
 uint16_t ally_base_HP;
};

/***********************
 * 0x0101 场地事件数据
 ***********************/
struct event_data_t
{
 uint32_t event_data;
};

/***********************
 * 0x0104 裁判警告数据
 ***********************/
struct referee_warning_t
{
 int8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
};

/***********************
 * 0x0105 飞镖发射相关数据
 ***********************/
struct dart_info_t
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
};

/***********************
 * 0x0201 机器人性能体系数据
 ***********************/
struct robot_status_t {
 uint8_t robot_id;
 uint8_t robot_level;

 uint16_t current_HP;
 uint16_t maximum_HP;

 uint16_t shooter_barrel_cooling_value;
 uint16_t shooter_barrel_heat_limit;
 uint16_t chassis_power_limit;

 uint8_t power_management_gimbal_output: 1;
 uint8_t power_management_chassis_output: 1;
 uint8_t power_management_shooter_output: 1;
};

/***********************
* 0x0202 实时底盘缓冲能量和射击热量数据
 ***********************/
struct power_heat_data_t
{
    uint16_t reserved1;
    uint16_t reserved2;

    float reserved3;

    uint16_t buffer_energy;

    uint16_t shooter_17mm_heat;
    uint16_t shooter_42mm_heat;
};


/***********************
 * 0x0203 机器人位置数据
 ***********************/
struct robot_pos_t
{
    float x;
    float y;
    float angle;
};

/***********************
 * 0x0204 机器人增益和底盘能量数据
 ***********************/
struct buff_t
{
 uint8_t recovery_buff;
 uint16_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
 uint8_t remaining_energy;
};

/***********************
 * 0x0206 伤害状态数据
 ***********************/
struct hurt_data_t
{
    uint8_t armor_id:4;
    uint8_t hurt_HP_deduction_reason:4;
};

/***********************
 * 0x0207 实时射击数据
 ***********************/
struct shoot_data_t
{
    uint8_t bullet_type;
    uint8_t shooter_number;

    uint8_t launching_frequency;

    float initial_speed;
};

/***********************
 * 0x0208 允许发弹量
 ***********************/
struct projectile_allowance_t
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
 uint16_t projectile_allowance_fortress;
};

/***********************
 * 0x0209 机器人 RFID 模块状态
 ***********************/
struct rfid_status_t
{
 uint32_t rfid_status;
 uint8_t rfid_status_2;
};

/***********************
 * 0x020A  飞镖选手端指令数据
 ***********************/
struct dart_client_cmd_t
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
};

/***********************
 * 0x020B 地面机器人位置数据
 ***********************/
struct ground_robot_position_t
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float reserved1;
 float reserved2;
};

/***********************
 * 0x020C 雷达标记进度数据
 ***********************/
struct radar_mark_data_t
{
 uint16_t mark_progress;
};

/***********************
 * 0x020D 哨兵自主决策信息同步
 ***********************/
struct sentry_info_t
{
 uint32_t sentry_info;
 uint16_t sentry_info_2;
};

/***********************
 * 0x020E 雷达自主决策信息同步
 ***********************/
struct radar_info_t
{
 uint8_t radar_info;
};

/***********************
 * 0x0301 机器人交互数据
 ***********************/
struct robot_interaction_data_t
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[112];
};

/***********************
* 0x0302 自定义控制器与机器人交互数据
 ***********************/
struct custom_robot_data_t
{
 uint8_t data[30];
};

/***********************
 * 0x0303 选手端小地图交互数据
 ***********************/
struct map_command_t
{
 float target_position_x;
 float target_position_y;
 uint8_t cmd_keyboard;
 uint8_t target_robot_id;
 uint16_t cmd_source;
};

/***********************
 * 0x0304 键鼠遥控数据
 ***********************/
struct remote_control_t
{
 int16_t mouse_x;
 int16_t mouse_y;
 int16_t mouse_z;
 int8_t left_button_down;
 int8_t right_button_down;
 uint16_t keyboard_value;
 uint16_t reserved;
};

/***********************
 * 0x0305 选手端小地图接收雷达数据
 ***********************/
struct map_robot_data_t
{
 uint16_t hero_position_x;
 uint16_t hero_position_y;
 uint16_t engineer_position_x;
 uint16_t engineer_position_y;
 uint16_t infantry_3_position_x;
 uint16_t infantry_3_position_y;
 uint16_t infantry_4_position_x;
 uint16_t infantry_4_position_y;
 uint16_t infantry_5_position_x;
 uint16_t infantry_5_position_y;
 uint16_t sentry_position_x;
 uint16_t sentry_position_y;
};

/***********************
* 0x0306 自定义控制器与选手端交互数据
 ***********************/
struct custom_client_data_t
{
 uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
};

/***********************
 * 0x0307 选手端小地图接收路径数据
 ***********************/
struct map_data_t
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
 uint16_t sender_id;
};

/***********************
 * 0x0308 选手端小地图接收机器人数据
 ***********************/
struct custom_info_t
{
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[30];
};

/***********************
 * 0x0309 自定义控制器接收机器人数据
 ***********************/
struct robot_custom_data_t
{
 uint8_t data[30];
};

/***********************
* 0x0310 机器人发送给自定义客户端的数据
 ***********************/
struct robot_custom_data_2_t
{
 uint8_t data[150];
};

/***********************
 * 0x0F01 设置图传出图信道
 ***********************/

/***********************
 * 0x0F02 查询当前出图信道
 ***********************/

/***********************
 * 0x0A01 对方机器人的位置坐标
 ***********************/

/***********************
 * 0x0A02 对方机器人的血量信息
 ***********************/

/***********************
 * 0x0A03 对方机器人的剩余发弹量信息
 ***********************/

/***********************
 * 0x0A04 对方队伍的宏观状态信息
 ***********************/

/***********************
 * 0x0A05 对方各机器人当前增益效果
 ***********************/

/***********************
 * 0x0A06 对方干扰波密钥
 ***********************/

#pragma pack(pop)


/***********************
 * 解析后提供给系统使用的数据
 ***********************/
struct RefereeInfo
{
 /******** 比赛状态 ********/
 uint8_t game_type = 0;
 uint8_t game_progress = 0;
 uint16_t stage_remain_time = 0;

 /******** 机器人状态 ********/
 uint8_t robot_id = 0;
 uint8_t robot_level = 0;

 uint16_t current_hp = 0;
 uint16_t max_hp = 0;

 uint16_t chassis_power_limit = 0;

 bool gimbal_output = false;
 bool chassis_output = false;
 bool shooter_output = false;

 /******** 功率与热量 ********/
 uint16_t buffer_energy = 0;

 uint16_t shooter_heat_17mm = 0;
 uint16_t shooter_heat_42mm = 0;

 uint16_t remaining_energy = 0;
 /******** 机器人位置 ********/
 float robot_x = 0;
 float robot_y = 0;
 float robot_angle = 0;

 /******** 射击信息 ********/
 uint8_t bullet_type = 0;
 uint8_t shooter_id = 0;
 uint8_t shoot_freq = 0;
 float bullet_speed = 0;

 /******** 伤害信息 ********/
 uint8_t hurt_armor_id = 0;
 uint8_t hurt_type = 0;

 /******** 自定义控制器信息 ********/
 int16_t J1 = 0;
 int16_t J2 = 0;
 int16_t J3 = 0;
 int16_t J4 = 0;
 int16_t J5 = 0;
 int16_t J6 = 0;

 /******** 数据更新标志 ********/
 bool updated = false;
};

extern RefereeInfo g_referee_info;


/***********************
 * Decode class
 ***********************/
class RefereeDecode
{
public:
    explicit RefereeDecode(RefereeProtocol& proto);

    static void onFrame(uint16_t cmd_id,
                        const uint8_t* data,
                        uint16_t len);

private:
};
#endif

#ifdef __cplusplus
extern "C"{
#endif

    void RefereeCallback(volatile uint8_t* buf, int len);

#ifdef __cplusplus
}
#endif

#endif