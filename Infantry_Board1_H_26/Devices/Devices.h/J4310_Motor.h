/**
 * @file D4310_Motor.h
 * @author ZS
 * @brief 
 * @version 0.1
 * @date 2024-12-20
 * 
 * @copyright Copyright (c)
 * 
 */
#ifndef __J4310_MOTOR_H
#define __J4310_MOTOR_H

#include "fdcan.h"
#include "main.h"
#include "typedef.h"
#include "Task_FdcanReceive.h"
#include "PID.h"

#define J4310_READID_PITCH 0x01
#define J4310_SENDID_Pitch 0x001 //控制Pitch轴电机
#define J4310_MaxV 200 //发送给电机的最大转速,单位rpm
#define J4310_MaxT 7 //发送给电机的最大扭矩，单位NM
#define J4310_ReductionRatio 10 //电机减速比

#define J4310_FIRSTANGLE 3800 /* 电机初始位置 */

#define J4310_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define Pi 3.14159265

#define J4310_getRoundAngle(rAngle) rAngle / J4310_mAngleRatio //机械角度与真实角度的比率

#define J4310_FunGroundInit        \
    {                              \
		  &J4310_setParameter,		   \
			&J4310_Enable,      \
			&J4310_Save_Pos_Zero,    \
			&J4310_getInfo,		   \
			&J4310_setTargetAngle, \
			&J4310_Reset,          \
			&Check_J4310,		   \
    }

typedef struct
{
		int16_t  state; 	   //读回来的电机状态
    float realAngle;  //算出来的机械角度（单位：度）
    float realSpeed;   //算出来的速度（单位：rpm）
    uint8_t temperatureMOS;    //读回来的电机MOS温度
		uint8_t temperatureRotor;  //读回来的电机线圈温度
	  float  torqueInit;       //读回来的电机扭矩
	  float  torque;           //算出来的电机扭矩
	  float  angleInit;   //读回来的机械角度
    float  speedInit;   //读回来的速度
		
    uint16_t lastAngle;  //上次的角度
	
    int32_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
	
	  float outPosition;   //输出位置
	  float outSpeed;      //输出速度
	  float outTorque;     //输出扭矩
	
    int16_t turnCount;   //转过的圈数
    float totalAngle;    //累积总共角度

    int8_t outKp;          //位置比例系数
    int8_t outKd;          //位置微分系数

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} J4310s_t;

extern J4310s_t J4310s_Pitch;   

typedef enum
{
    //需要注意与报文接受函数处对应。即
    //J4310_PITCH_Right = 0,
    J4310_PITCH = 0,
} J4310Name_e;


typedef struct
{
	  void (*J4310_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
    void (*J4310_Enable)(void);
	  void (*J4310_Save_Pos_Zero)(void);
		void (*J4310_getInfo)(Fdcan_Export_Data_t RxMessage);
    void (*J4310_setTargetAngle)(J4310s_t *J4310, int32_t angle);
    void (*J4310_Reset)(J4310s_t *J4310);
	  void (*Check_J4310)(void);
} J4310_Fun_t;

extern J4310s_t J4310s_Pitch; //ID为8
extern J4310_Fun_t J4310_Fun;

#endif /* __J4310_MOTOR_H */
