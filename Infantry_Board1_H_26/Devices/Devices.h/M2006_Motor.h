/**
 * @file M2006_Motor.h
 * @author Why
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H

#include "fdcan.h"
#include "main.h"
#include "typedef.h"
//#include "Task_CanReceive.h"
#include "PID.h"

#define M2006_READID_START 0x207
#define M2006_READID_END 0x207
#define M2006_DIAL_ID 0x207
#define M2006_MaxOutput 10000 //发送给电机的最大控制值
#define M2006_ReductionRatio 36 //电机减速比
#define M2006_FIRSTANGLE 3800 /* 电机初始位置 */

typedef enum
{
    Dial_Motor = 0,    //拨弹电机
    Total_Num,
} M2006_MotorName;

#define M2006_FunGroundInit \
    {                       \
        &M2006_getInfo,     \
        &M2006_setCurrent   \
    }

typedef struct
{
    uint16_t realAngle; //读回来的机械角度
    int16_t realSpeed;  //读回来的速度
    int16_t realTorque; //读回来的实际转矩

    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度

    uint16_t lastAngle; //上次的角度
    int32_t totalAngle; //累积总共角度
    int16_t turnCount;  //转过的圈数

    int16_t outCurrent;      //输出电流
    int16_t inneroutCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M2006s_t;

typedef struct
{
    void (*M2006_getInfo)(Fdcan_Export_Data_t RxMessage);
    void (*M2006_setCurrent)(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);
} M2006_FUN_t;

void M2006_Init(void);
extern M2006_FUN_t M2006_FUN;
extern M2006s_t M2006_Array[Total_Num];                 //四个2006电机的结构体数组

#endif /* __M2006_MOTOR_H */
