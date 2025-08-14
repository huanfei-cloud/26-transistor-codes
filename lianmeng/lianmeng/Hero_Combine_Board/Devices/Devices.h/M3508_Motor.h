/**
 * @file M3508_Motor.h
 * @author Why
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
#include "fdcan.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>

/* 记录M3508各个电机ID
*/
#define M3508_READID_CHASSIS_START 0x201
#define M3508_READID_CHASSIS_END 0x204
#define M3508_READID_SHOOT_START 0x205
#define M3508_READID_SHOOT_END 0x207
#define M3508_SENDID_Chassis 0x200   //控制底盘电机
#define M3508_SENDID_Fric_Dial 0x1FF //控制摩擦轮和拨盘电机
#define M3508_MaxOutput 16384        //发送给电机的最大控制值
#define M3508_CurrentRatio 819.2f    //16384/20A = 819.2->1A
#define M3508_ReductionRatio 3591/187 //3508电机减速比

#define M3508_FunGroundInit          \
    {                                \
        &M3508_getInfo,              \
		&M3508_setCurrent,           \
		&Check_WheelM3508,   	     \
    }

/******关键设定：pid跟3508电机数组的前四个是一一对应的四个轮子
*******这会在其他地方用到************************************/
	
	/**
  * @brief  用以区别3508电机数组各个元素的归属
  * @param  Chassis_Right_Front	右前轮
  *			Chassis_Left_Front  左前轮
  *			Chassis_Left_Back	左后轮
  *			Chassis_Right_Back	右后轮
  *			FricL_Wheel			左摩擦轮
  *			FricR_Wheel			右摩擦轮
	*			FricU_Wheel			上摩擦轮
  *			DialMotor			拨弹电机
  */
typedef enum
{
	FricL_Wheel=0,
	FricR_Wheel,
	FricU_Wheel,
	Dial_Motor,

}M3508_MotorName;

typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed;  //目标速度
    int64_t targetAngle; //目标角度
    uint16_t lastAngle;   //上次的角度
    int64_t totalAngle;   //累积总共角度
    int16_t turnCount;    //转过的圈数

    int16_t outCurrent;   //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M3508s_t;

extern M3508s_t M3508_Array[8];

typedef struct
{
    void (*M3508_getInfo)(Fdcan_Export_Data_t RxMessage);
	void (*M3508_setCurrent)(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);
	void (*Check_WheelM3508)(void);
} M3508_FUN_t;

extern M3508_FUN_t M3508_FUN;        

#endif /*__M3508_MOTOR_H*/
