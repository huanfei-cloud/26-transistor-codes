/**
 * @file D4310_Motor.h
 * @author ZS
 * @brief 
 * @version 0.1
 * @date 2024-12-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __J4310_MOTOR_H
#define __J4310_MOTOR_H

#include "fdcan.h"
#include "main.h"
#include "typedef.h"
#include "Task_FdcanReceive.h"
#include "PID.h"

#define J4310_READID_START 0x01
#define J4310_READID_END 0x01
#define J4310_MaxV 200 //发送给电机的最大转速,单位rpm
#define J4310_MaxT 7 //发送给电机的最大扭矩，单位NM
#define J4310_ReductionRatio 10 //电机减速比
//#define M2006_LOADANGLE		42125			/* 电机拨一个弹需要转的角度数  6*8191 （7孔拨弹）*/

//#define M2006_LOADCIRCLE	5			/* 电机拨一个弹需要转的圈数 */
//#define M2006_LOADSPEED		1800		/* 电机拨弹时的转速 */
#define J4310_FIRSTANGLE 3800 /* 电机初始位置 */

#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率

#define M6020_getRoundAngle(rAngle) rAngle / M6020_mAngleRatio //机械角度与真实角度的比率

#define M6020_FunGroundInit        \
    {                              \
		&M6020_setVoltage,		   \
			&M6020_getInfo,		   \
			&M6020_setTargetAngle, \
			&M6020_Reset,          \
			&Check_M6020,		   \
    }

typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int32_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际转矩电流
    uint8_t temperture;  //读回来的电机温度
    uint16_t lastAngle;  //上次的角度
	
    int32_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
	
    int16_t turnCount;   //转过的圈数
    float totalAngle;  //累积总共角度

    int16_t outCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M6020s_t;

typedef enum
{
    //需要注意与报文接受函数处对应。即
    //M6020_PITCH_Right = 0,
    M6020_PITCH = 0,
    M6020_YAW,
} M6020Name_e;

typedef struct
{
	void (*M6020_setVoltage)(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
    void (*M6020_getInfo)(Fdcan_Export_Data_t RxMessage);
    void (*M6020_setTargetAngle)(M6020s_t *M6020, int32_t angle);
    void (*M6020_Reset)(M6020s_t *m6020);
	void (*Check_M6020)(void);
} M6020_Fun_t;

extern M6020s_t M6020s_Yaw;   //ID为1
extern M6020s_t M6020s_Pitch; //2
extern M6020_Fun_t M6020_Fun;

#endif /* __M3508_MOTOR_H */
