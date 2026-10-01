//
// Created by 20852 on 2025/9/9.
//

#include "bsp_can.h"
#include <concepts>
#include <cstring>
#include "debug_vars.h"

//前向声明：DM电机数据转换辅助函数
static float dm_uint_to_float(int x_int, float x_min, float x_max, int bits);
static int   dm_float_to_uint(float x_float, float x_min, float x_max, int bits);
static void  update_dm_motor_info(DM_motor_info* motor, const uint8_t* data);

//外部CAN句柄 如果有更多的hcan句柄同样在这里进行定义
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//创建不同电机的结构体变量
DJI_motor_info motor_1;
DJI_motor_info motor_2;
DJI_motor_info motor_3;
DJI_motor_info motor_4;
DJI_motor_info motor_5;
DJI_motor_info motor_6;

DM_motor_info DM_motor_1;
// 达妙(DM)六关节机械臂。反馈仲裁 ID 为各电机独立配置的 Master ID，
// 接收时通过数据 D[0] 中的电机 ID 识别关节。
DM_motor_info DM_motor_J0;  // J0 底部 yaw，CAN ID=0x02
DM_motor_info DM_motor_J1;  // J1 底部 pitch，CAN ID=0x03
DM_motor_info DM_motor_J2;  // J2 roll，CAN ID=0x04
DM_motor_info DM_motor_J3;  // J3 末端 yaw，CAN ID=0x05
DM_motor_info DM_motor_J4;  // J4 pitch/yaw，CAN ID=0x06
DM_motor_info DM_motor_J5;  // J5 pitch/yaw，CAN ID=0x07
// 最近一次从各电机收到反馈时使用的 Master ID，便于核对电机上位机配置。
volatile uint16_t DM_feedback_master_id[6] = {};

LK_motor_info LK_motor_1;
LK_motor_info LK_motor_2;
LK_motor_info LK_motor_3;
//IMU数据结构体变量
imu_data_info imu_data_chassis;

//底盘数据结构体变量
chassis_data chassis;

//BSP_CAN相关内容初始化
void bsp_can::bsp_can_init()
{
    static bsp_can can;
    // 1. 配置 CAN 过滤器
    can.BSP_CAN_FilterConfig();

    // 2. 启动 CAN 外设
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    // 如果有其他 CAN 外设，也在这里启动

    // 3. 激活 CAN 接收中断 (当 FIFO0 中有新消息时触发)
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler(); // 激活中断失败
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler(); // 激活中断失败
    }
    // 如果有其他 CAN 外设，也在这里激活中断
}

//CAN过滤器配置，配置了CAN1与CAN2
void bsp_can::BSP_CAN_FilterConfig()
{
    CAN_FilterTypeDef filter;

    /* ------------ CAN1：过滤器 0-13 全部接收 ------------ */
    filter.FilterActivation       = ENABLE;
    filter.FilterMode             = CAN_FILTERMODE_IDMASK;
    filter.FilterScale            = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment   = CAN_FILTER_FIFO0;

    filter.FilterIdHigh           = 0x0000;
    filter.FilterIdLow            = 0x0000;
    filter.FilterMaskIdHigh       = 0x0000;
    filter.FilterMaskIdLow        = 0x0000;

    filter.FilterBank             = 0;       // CAN1 的第一个过滤器组
    filter.SlaveStartFilterBank   = 14;      // 14 之后给 CAN2 用

    HAL_CAN_ConfigFilter(&hcan1, &filter);


    /* ------------ CAN2：过滤器 14-27 全部接收 ------------ */
    filter.FilterBank = 14;                 // CAN2 的第一个过滤器组
    HAL_CAN_ConfigFilter(&hcan1, &filter);  // 注意：必须用 hcan1 配置 CAN2 过滤器
}

//DJI电机CAN命令，ID：1--4
HAL_StatusTypeDef bsp_can::BSP_CAN2_DJIMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x200; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor1 >> 8;
    TxData[1] = motor1;
    TxData[2] = motor2 >> 8;
    TxData[3] = motor2;
    TxData[4] = motor3 >> 8;
    TxData[5] = motor3;
    TxData[6] = motor4 >> 8;
    TxData[7] = motor4;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//DJI电机CAN命令，ID：5--8
HAL_StatusTypeDef bsp_can::BSP_CAN2_DJIMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x1FF; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor5 >> 8;
    TxData[1] = motor5;
    TxData[2] = motor6 >> 8;
    TxData[3] = motor6;
    TxData[4] = motor7 >> 8;
    TxData[5] = motor7;
    TxData[6] = motor8 >> 8;
    TxData[7] = motor8;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}

//DJI电机CAN命令，ID：9--11
HAL_StatusTypeDef bsp_can::BSP_CAN2_DJIMotorCmdNine2Eleven(int16_t motor9,int16_t motor10,int16_t motor11) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x2FF; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor9 >> 8;
    TxData[1] = motor9;
    TxData[2] = motor10 >> 8;
    TxData[3] = motor10;
    TxData[4] = motor11 >> 8;
    TxData[5] = motor11;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}

//DM电机失能命令
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorDisableCmd(uint16_t ID, uint16_t mode) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 本机械臂电机沿用已验证过的模式偏移寻址方式。
    TxHeader.StdId = ID + mode;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFD;

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//DM电机使能命令
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorEnableCmd(uint16_t ID, uint16_t mode) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 本机械臂电机沿用已验证过的模式偏移寻址方式。
    TxHeader.StdId = ID + mode;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFC;

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// DM 状态刷新：失能状态下使用 0x7FF / 0xCC 主动请求一次标准状态反馈。
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorRefreshStatusCmd(uint16_t ID) {
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    TxHeader.StdId = 0x7FFU;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8U;

    TxData[0] = static_cast<uint8_t>(ID & 0xFFU);
    TxData[1] = static_cast<uint8_t>((ID >> 8U) & 0xFFU);
    TxData[2] = 0xCCU;
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//DM电机位速闭环命令
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorPositionCmd(int16_t ID, float position, float velocity) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x100 + ID; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    memcpy(&TxData[0], &position, 4);
    memcpy(&TxData[4], &velocity, 4);

    //将信息推送到邮箱
    //CAN1 仅有 3 个发送邮箱，主循环连续发送 6 帧时，第 4~6 帧会因无空闲邮箱返回
    //HAL_BUSY 而被丢弃(表现为 J3/J4/J5 不动)；此处忙等一个空闲邮箱再发送
    HAL_StatusTypeDef status;
    uint32_t retry = 0;
    do {
        status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    } while (status == HAL_BUSY && (++retry < 10000));

    return status;
}

//DM电机速度闭环命令
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorVelocityCmd(int16_t ID, float velocity) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x200 + ID; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    auto* vbuf =(uint8_t*)&velocity;

    //数据填充
    //memcpy(&TxData[0], &velocity, 4);
    TxData[0] = *vbuf;
    TxData[1] = *(vbuf+1);
    TxData[2] = *(vbuf+2);
    TxData[3] = *(vbuf+3);

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//DM电机MIT模式控制命令 (力矩+位置+速度前馈)
HAL_StatusTypeDef bsp_can::BSP_CAN1_DMMotorMitCmd(int16_t ID, float pos, float vel, float kp, float kd, float torq) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //MIT模式: 发送ID = 电机ID + MIT_MODE(0x000)
    TxHeader.StdId = ID + DM_MIT_MODE;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //将物理量转换为原始整数值
    uint16_t pos_int = dm_float_to_uint(pos,  DM_P_MIN, DM_P_MAX, 16);
    uint16_t vel_int = dm_float_to_uint(vel,  DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_int  = dm_float_to_uint(kp,   DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int  = dm_float_to_uint(kd,   DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t tor_int = dm_float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    //MIT模式数据帧格式: pos(16b)|vel(12b)|kp(12b)|kd(12b)|tor(12b)
    TxData[0] = (pos_int >> 8) & 0xFF;          //位置高8位
    TxData[1] =  pos_int & 0xFF;                 //位置低8位
    TxData[2] = (vel_int >> 4) & 0xFF;           //速度高8位
    TxData[3] = ((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);  //速度低4位 + kp高4位
    TxData[4] =  kp_int & 0xFF;                  //kp低8位
    TxData[5] = (kd_int >> 4) & 0xFF;            //kd高8位
    TxData[6] = ((kd_int & 0x0F) << 4) | ((tor_int >> 8) & 0x0F);  //kd低4位 + 力矩高4位
    TxData[7] =  tor_int & 0xFF;                 //力矩低8位

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机失能命令            0x80
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorCloseCmd(uint16_t ID) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0x80;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机使能命令            0x88
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorStartCmd(uint16_t ID) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0x88;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机状态2读取命令        0x9C
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorReadStatus2Cmd(uint16_t ID) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充，其余字节按照协议保持为0
    TxData[0] = 0x9C;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机转矩命令            0xA1
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorTorqueCmd(uint16_t ID, int16_t current) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA1;   // 命令字：力矩控制
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;

    TxData[4] = current;         // 力矩低字节
    TxData[5] = current >> 8;  // 力矩高字节

    TxData[6] = 0x00;
    TxData[7] = 0x00;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机速度命令            0xA2
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorVelocityCmd(uint16_t ID, int32_t velocity) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA2;   // 命令字：速度控制
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;

    TxData[4] = velocity;          // 最低字节
    TxData[5] = velocity >> 8;   // 次低字节
    TxData[6] = velocity >> 16;  // 次高字节
    TxData[7] = velocity >> 24;  // 最高字节

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机增量位置命令         0xA7
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorIncrePosCmd(uint16_t ID, int32_t degree) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA7;   // 命令字：增量位置控制
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;

    TxData[4] = degree;          // 最低字节
    TxData[5] = degree >> 8;   // 次低字节
    TxData[6] = degree >> 16;  // 次高字节
    TxData[7] = degree >> 24;  // 最高字节

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//LK电机增量位置命令（含限速） 0xA8
HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorIncrePosVelRestrictCmd(uint16_t ID, int32_t degree, uint32_t velocity) {
    if (ID < 1 || ID > 32) {
        return HAL_ERROR;
    }

    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader = {};
    uint8_t TxData[8] = {};
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x140 + ID; //标准标识符：0x140 + 电机ID
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA8;   // 命令字：增量位置限速控制
    TxData[1] = 0x00;

    TxData[2] = velocity;
    TxData[3] = velocity >> 8;

    TxData[4] = degree;        // 最低字节
    TxData[5] = degree >> 8;   // 次低字节
    TxData[6] = degree >> 16;  // 次高字节
    TxData[7] = degree >> 24;  // 最高字节

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//上板对下板发送遥控器控制命令 0x301
HAL_StatusTypeDef bsp_can::BSP_CAN1_SendRemoteControlCmd(int16_t X,int16_t Y,int16_t Z,uint8_t s2) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x301; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = X >> 8;
    TxData[1] = X;
    TxData[2] = Y >> 8;
    TxData[3] = Y;
    TxData[4] = Z >> 8;
    TxData[5] = Z;
    TxData[6] = s2;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//辅助函数：更新电机累计角度
void update_motor_total_angle(DJI_motor_info* motor, uint16_t new_ecd)
{
    // 首次初始化处理
    if (!motor->inited) {
        motor->last_angle = new_ecd;
        motor->round_count = 0;
        motor->total_angle = (int32_t)new_ecd;
        motor->inited = 1;
        return;
    }

    // 使用有符号计算差分，避免 uint16 溢出问题
    int32_t diff = static_cast<int32_t>(new_ecd) - static_cast<int32_t>(motor->last_angle);

    // 过零检测 (当差值超过半圈认为跨了圈)
    if (diff > 4096) {
        // new_ecd 小， last 大 -> 实际向上跨过 0 点， round_count 增加
        motor->round_count--;
    }
    else if (diff < -4096) {
        motor->round_count++;
    }

    // 计算连续编码器读数（ticks）
    motor->total_angle = motor->round_count * (int32_t)8192 + (int32_t)new_ecd;

    // 最后更新 last_angle
    motor->last_angle = new_ecd;
}

//辅助函数：将DM电机原始整数值转换为物理量 (float)
static float dm_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//辅助函数：将物理量转换为DM电机原始整数值
static int dm_float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

//辅助函数：解析达妙(DM)电机反馈数据帧 (8字节标准格式)
static void update_dm_motor_info(DM_motor_info* motor, const uint8_t* data)
{
    motor->id        =  data[0] & 0x0F;                      // ID在低4位
    motor->err       =  data[0] >> 4;                        // 状态/错误码在高4位：0失能，1使能，8~E故障
    motor->pos_raw   = (data[1] << 8) | data[2];             // 位置 (16bit)
    motor->vel_raw   = (data[3] << 4) | (data[4] >> 4);      // 速度 (12bit)
    motor->torque_raw= ((data[4] & 0x0F) << 8) | data[5];    // 力矩 (12bit)
    motor->temp_mos  =  data[6];                             // MOS温度

    //转换为物理量
    motor->pos    = dm_uint_to_float(motor->pos_raw,    DM_P_MIN, DM_P_MAX, 16);
    motor->vel    = dm_uint_to_float(motor->vel_raw,    DM_V_MIN, DM_V_MAX, 12);
    motor->torque = dm_uint_to_float(motor->torque_raw, DM_T_MIN, DM_T_MAX, 12);

    // 最后更新状态，任务层看到 rx_count 增加时，前面的反馈字段已经完整。
    motor->last_update_tick = HAL_GetTick();
    motor->rx_count++;
}

// 达妙反馈帧的仲裁 ID 是可独立配置的 Master ID，不能假定恒为 CAN_ID + 0x10。
// 数据 D[0] 的低四位才是发送反馈的电机 CAN ID。
static bool try_update_dm_motor_info(uint32_t master_id, uint32_t dlc,
                                     const uint8_t* data)
{
    if (master_id > 0x7FFU || dlc != 8U) return false;

    const uint8_t motor_id = data[0] & 0x0FU;
    if (motor_id < 0x02U || motor_id > 0x07U) return false;

    const uint8_t joint = static_cast<uint8_t>(motor_id - 0x02U);
    DM_motor_info* const motors[6] = {
        &DM_motor_J0, &DM_motor_J1, &DM_motor_J2,
        &DM_motor_J3, &DM_motor_J4, &DM_motor_J5
    };
    update_dm_motor_info(motors[joint], data);
    DM_feedback_master_id[joint] = static_cast<uint16_t>(master_id);
    return true;
}

//辅助函数：更新瓴控电机反馈信息
void update_lk_motor_info(LK_motor_info* motor, const uint8_t* data)
{
    motor->temp = static_cast<int8_t>(data[1]);
    motor->torque_current = static_cast<int16_t>((data[3] << 8) | data[2]);
    motor->rotor_speed = static_cast<int16_t>((data[5] << 8) | data[4]);
    motor->rotor_angle = static_cast<uint16_t>((data[7] << 8) | data[6]);
}

//FIFO0接收回调函数，分CAN1，CAN2
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // 在这里解析RxData，更新电机状态等
        if(hcan->Instance == CAN2)
        {
            switch(RxHeader.StdId)
            {
                case 0x201://此处仅接收了id为0x201电机的报文
                {
                    motor_1.rotor_angle    = ((RxData[0] << 8) | RxData[1]);
                    motor_1.rotor_speed    = ((RxData[2] << 8) | RxData[3]);
                    motor_1.torque_current = ((RxData[4] << 8) | RxData[5]);
                    motor_1.temp           =   RxData[6];
                    break;
                }
                case 0x202: {
                    motor_2.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_2.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_2.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_2.temp          =   RxData[6];
                    break;
                }
                case 0x203: {
                    motor_3.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_3.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_3.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_3.temp          =   RxData[6];
                    break;
                }
                case 0x204: {
                    motor_4.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_4.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_4.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_4.temp          =   RxData[6];
                    break;
                }
                case 0x205: {
                    motor_5.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_5.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_5.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_5.temp          =   RxData[6];
                    update_motor_total_angle(&motor_5, motor_5.rotor_angle);

                    debug_angle1 = motor_5.total_angle;
                    break;
                }
                case 0x206: {
                    motor_6.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_6.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_6.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_6.temp          =   RxData[6];
                    update_motor_total_angle(&motor_6, motor_6.rotor_angle);

                    debug_angle2 = motor_6.total_angle;
                    break;
                }
                default: break;
            }
        }
        else if(hcan->Instance == CAN1) {
            switch(RxHeader.StdId) {
                //处理ID为1、2、3的瓴控电机反馈
                case 0x141:
                case 0x142:
                case 0x143: {
                    LK_motor_info* motor = nullptr;
                    if (RxHeader.StdId == 0x141) {
                        motor = &LK_motor_1;
                    }
                    else if (RxHeader.StdId == 0x142) {
                        motor = &LK_motor_2;
                    }
                    else {
                        motor = &LK_motor_3;
                    }

                    switch (RxData[0]) {
                        //状态2和控制命令的回复数据格式相同
                        case 0x9C:
                        case 0xA1:
                        case 0xA2:
                        case 0xA7:
                        case 0xA8: {
                            update_lk_motor_info(motor, RxData);

                            if (RxHeader.StdId == 0x141) {
                                debug_angle1 = LK_motor_1.rotor_angle;
                            }
                            else if (RxHeader.StdId == 0x142) {
                                debug_angle2 = LK_motor_2.rotor_angle;
                            }
                            break;
                        }
                        default:break;
                    }
                    break;
                }
                //处理IMU数据
                case 0x401: {
                    // 高字节在前，恢复 int16_t
                    auto roll_int  = static_cast<int16_t>((RxData[0] << 8) | RxData[1]);
                    auto pitch_int = static_cast<int16_t>((RxData[2] << 8) | RxData[3]);
                    auto yaw_int   = static_cast<int16_t>((RxData[4] << 8) | RxData[5]);

                    // 转回 float
                    imu_data_chassis.roll  = roll_int  / 100.0f;
                    imu_data_chassis.pitch = pitch_int / 100.0f;
                    imu_data_chassis.yaw   = yaw_int   / 100.0f;
                    break;
                }
                //处理底盘数据
                case 0x501: {
                    chassis.spinvelocity = static_cast<int16_t>((RxData[0] << 8) | RxData[1]);
                    break;
                }
                default: {
                    try_update_dm_motor_info(RxHeader.StdId, RxHeader.DLC, RxData);
                    break;
                }
            }
        }
    }
}

// C接口封装
extern "C" {
    static bsp_can can;
    void BSP_CAN_Init() {
        can.bsp_can_init();
    }

    HAL_StatusTypeDef bsp_can2_djimotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        return can.BSP_CAN2_DJIMotorCmd(motor1, motor2, motor3, motor4);
    }

    HAL_StatusTypeDef bsp_can2_djimotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
        return can.BSP_CAN2_DJIMotorCmdFive2Eight(motor5, motor6, motor7, motor8);
    }

    HAL_StatusTypeDef bsp_can2_djimotorcmdnine2eleven(int16_t motor9,int16_t motor10,int16_t motor11) {
        return can.BSP_CAN2_DJIMotorCmdNine2Eleven(motor9,motor10,motor11);
    }

    HAL_StatusTypeDef bsp_can1_dmmotordisablecmd(uint16_t ID, uint16_t mode) {
        return can.BSP_CAN1_DMMotorDisableCmd(ID,mode);
    }

    HAL_StatusTypeDef bsp_can1_dmmotorenablecmd(uint16_t ID, uint16_t mode) {
        return can.BSP_CAN1_DMMotorEnableCmd(ID,mode);
    }

    HAL_StatusTypeDef bsp_can1_dmmotorpositioncmd(int16_t ID, float position,float velocity) {
        return can.BSP_CAN1_DMMotorPositionCmd(ID,position,velocity);
    }

    HAL_StatusTypeDef bsp_can1_dmmotorvelocitycmd(int16_t ID,float velocity) {
        return can.BSP_CAN1_DMMotorVelocityCmd(ID,velocity);
    }

    HAL_StatusTypeDef bsp_can1_dmmotormitcmd(int16_t ID, float pos, float vel, float kp, float kd, float torq) {
        return can.BSP_CAN1_DMMotorMitCmd(ID, pos, vel, kp, kd, torq);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorclosecmd(uint16_t ID) {
        return can.BSP_CAN1_LKMotorCloseCmd(ID);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorstartcmd(uint16_t ID) {
        return can.BSP_CAN1_LKMotorStartCmd(ID);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorreadstatus2cmd(uint16_t ID) {
        return can.BSP_CAN1_LKMotorReadStatus2Cmd(ID);
    }

    HAL_StatusTypeDef bsp_can1_lkmotortorquecmd(uint16_t ID, int16_t torque) {
        return can.BSP_CAN1_LKMotorTorqueCmd(ID, torque);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorvelocitycmd(uint16_t ID, int32_t velocity) {
        return can.BSP_CAN1_LKMotorVelocityCmd(ID, velocity);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorincreposcmd(uint16_t ID, int32_t degree) {
        return can.BSP_CAN1_LKMotorIncrePosCmd(ID, degree);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorincreposvelrestrictcmd(uint16_t ID, int32_t degree,uint32_t velocity) {
        return can.BSP_CAN1_LKMotorIncrePosVelRestrictCmd(ID, degree, velocity);
    }

    HAL_StatusTypeDef bsp_can1_sendremotecontrolcmd(int16_t X,int16_t Y,int16_t Z,uint8_t s2) {
        return can.BSP_CAN1_SendRemoteControlCmd(X,Y,Z,s2);
    }
}
