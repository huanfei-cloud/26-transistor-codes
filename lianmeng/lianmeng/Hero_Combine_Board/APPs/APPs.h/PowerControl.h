/**
 * @file PowerControl.c
 * @author Why
 * @brief 控制输出功率在限制功率之下
 * @version 0.1
 * @date 2023-08-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H

#include "main.h"
#include "BSP_Fdcan.h"

/******关键设定：pid跟3508电机数组的前四个是一一对应的四个轮子
*******这会在其他地方用到************************************/

/* 实际功率经验公式参数，需要实际测量拟合，公式：Pin = Pm + Kw * w^2 +Kτ * τ^2 + A */
#define PowerBuffer_ID 0x2E
#define PowerLimit_ID 0x2F
#define PowerFeedback_ID 0x30
/* 转矩电流读取值跟实际转矩之间的转换常数，单位N·m/A ，公式：τ = TC * Iτ*/
#define Torque_Constant 0.0000190703f
#define It_Constant 52437.56f
/* 机械功率常数，公式：Pm = τ*w / 9.55 = τ*w*MPC */
#define MachinePower_Cnstant 0.104712042
/* 实际功率经验公式参数，需要实际测量拟合，公式：Pin = Pm + Kw * w^2 +Kτ * τ^2 + A */
#define Kw_d 0
#define Kt_d 0
#define A_d 0

#define Div_2Kt_d 0

#define PowerControl_FunGroundInit      \
    {                                   \
        &PowerControl_Handle,           \
		&PowerControl_MsgSend,			\
		&PowerControl_MsgRec,			\
		&PowerContol_MsgRenew,			\
    }

#define PowerControl_DataGroundInit \
    {                      \
        300,               \
        150,               \
        3,                 \
		0,                 \
        0,                 \
        0,                 \
        120,               \
        120,               \
        55,                \
    }

typedef struct
{
    /* 设定的值 */
    int16_t DischargeLimit;        // 超级电容放电限制功率
    uint16_t ChargeLimit;          // 超级电容充电限制功率
    uint16_t ControlBit;           // 超级电容控制位，第一位是开关，第二位是显示开关

    /* 传回的值 */
    float Cap_U;                   // 超级电容组电压
    float Cap_I;                   // 超级电容组电流
    union                          // 超级电容组状态，详见超级电容技术说明
    {
        int16_t Cap_Status;
        struct
        {
            uint16_t warning : 1;           //报警
            uint16_t cap_v_over : 1;        //电容过压
            uint16_t cap_i_over : 1;        //电容过流
            uint16_t cap_v_low : 1;         //电容欠压
            uint16_t bat_v_low : 1;         //裁判系统欠压
            uint16_t can_receive_miss : 1;  //未读到CAN通信数据
        } bit;
    }Cap_Status_t;

    /* 计算出的值 */
    uint16_t Power_Available;      // 超级电容放电加电池供电的最大功率

    /* 裁判系统的值 */
    uint16_t Power_Limit;          // 限制功率
    uint16_t PoweBuffer;           // 裁判系统测量出的功率值
} PowerControl_Data_t;

typedef struct
{
    void (*PowerControl_Handle)(void);
	void (*PowerControl_MsgSend)(void);
	void (*PowerControl_MsgRec)(Fdcan_Export_Data_t RxMessage);
	void (*PowerContol_MsgRenew)(void);
} PowerControl_Fun_t;

extern PowerControl_Fun_t PowerControl_Fun;
extern PowerControl_Data_t PowerControl_Data;

#endif /*__POWERCONTROL_H*/
