/**
 * @file Dial.h
 * @author Why
 * @brief 拨弹盘电机控制
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __DIAL_H
#define __DIAL_H

#include "Shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "FuzzyPID.h"
#include "PID.h"
#include "FeedForward.h"
#include "stm32f4xx_hal.h"

/**
 * @brief  拨弹盘开关
 * @param
 */

typedef enum
{
	Dial_Off = 0,
	Dial_On,
	Dial_Back,
} Dial_On_Off;

/**
 * @brief  发射模式
 * @param
 */
typedef enum
{
	Single_Shoot = 0,
	Continuous_Shoot,
	No_Shoot,
} Shoot_Modes;

/* 拨出一个弹丸，电机需要转动的角度，映射为360°-8192360 */
#define Angle_DialOneBullet_42mm 31462.536898 // 8192/6*19.2
#define Angle_DialOneBullet_17mm 36864		  // 1024*36

/* Dial函数数据以及函数的接口结构体 */
#define Dial_DataGroundInit \
	{                       \
		0,                  \
		100,                \
		5000,               \
		0,                  \
		0,                  \
		0,                  \
		-100,               \
		Single_Shoot,       \
		Dial_Off,           \
	}

#define Dial_FunGroundInit    \
	{                         \
		&Dial_Processing_2,   \
		&Dial_Update_Angel,   \
		&Dial_Back_OneBullet, \
		&Dial_OneBullet,      \
	}

typedef struct Dial_Data_t
{
	uint8_t Number_ToBeFired; // 需要打击的弹丸数量
	uint16_t Shoot_Interval;  // 发射间隔
	uint16_t Shoot_Period;	  // 连发的时间
	uint32_t Time_NextShoot;  // 下次射击的时间，即系统时间超过这个值才能射击
	uint32_t Time_StopShoot;  // 连发停止发射的时间
	uint16_t Bullet_Dialed;	  // 已经打出的子弹
	int16_t Speed_Dial;		  // 连发的拨盘电机速度
	Shoot_Modes Shoot_Mode;	  // 发射模式
	Dial_On_Off Dial_Switch;  // 拨盘开关状态
} Dial_Data_t;

typedef struct Dial_Fun_t
{
	void (*Dial_Processing_2)();
	void (*Dial_Update_Angel)(bool Fric_ReadyOrNot);
	void (*Dial_Back_OneBullet)();
	void (*Dial_OneBullet)();

} Dial_Fun_t;

extern Dial_Fun_t Dial_Fun;
extern Dial_Data_t Dial_Data;

#endif /*__DIAL_H*/
