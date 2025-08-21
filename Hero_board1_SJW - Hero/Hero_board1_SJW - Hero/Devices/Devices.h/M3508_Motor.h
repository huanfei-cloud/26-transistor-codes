/**
 * @file M3508_Motor.h
 * @author Why,SJW
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "PID.h"
#include "can.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* 记录M3508各个电机ID
 */
#define M3508_SENDID_Fric_Dial_1 0x200
#define M3508_SENDID_Fric_Dial_2 0x1FF
#define M3508_READID_START_1 0x201      //前3个摩擦轮和拨弹盘的ID
#define M3508_READID_END_1 0x204				
#define M3508_READID_START_2 0x205			//后三个摩擦轮ID
#define M3508_READID_END_2 0x207
#define M3508_MaxOutput 16384           // 发送给电机的最大控制值
#define M3508_CurrentRatio 819.2f       // 16384/20A = 819.2->1A
#define M3508_ReductionRatio 3591 / 187 // 3508电机减速比

#define M3508_FunGroundInit \
    {                       \
        &M3508_getInfo,     \
        &M3508_setCurrent,  \
    }

/**
 * @brief  用以区别3508电机数组各个元素的归属
 * @param
 */
typedef enum
{
    Fric_Front_1 = 0,
    Fric_Front_2,
    Fric_Front_3,
    Dial_Wheel,
    Fric_Back_1,
    Fric_Back_2,
    Fric_Back_3,
    TotalNum,
} M3508_MotorName;

typedef struct
{
    uint16_t realAngle;  // 读回来的机械角度
    int16_t realSpeed;   // 读回来的速度
    int16_t realCurrent; // 读回来的实际电流
    uint8_t temperture;  // 读回来的电机温度

    int16_t targetSpeed;  // 目标速度
    uint16_t targetAngle; // 目标角度
    uint16_t lastAngle;   // 上次的角度
    int32_t totalAngle;   // 累积总共角度
    int16_t turnCount;    // 转过的圈数

    int16_t outCurrent; // 输出电流

    uint8_t InfoUpdateFlag;   // 信息读取更新标志
    uint16_t InfoUpdateFrame; // 帧率
    uint8_t OffLineFlag;      // 设备离线标志
} M3508s_t;

extern M3508s_t M3508_Array[TotalNum];

typedef struct
{
    void (*M3508_getInfo)(Can_Export_Data_t RxMessage);
    void (*M3508_setCurrent)(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);
} M3508_FUN_t;

extern M3508_FUN_t M3508_FUN;

#endif /*__M3508_MOTOR_H*/
