/**
 * @file Dial.h
 * @author Why
 * @brief 处理拨盘电机的控制问题
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
 
/**
  * @brief  拨弹电机的开关
  * @param  
  */
typedef enum
{
	Dial_Off  = 0,
	Dial_On,
	Dial_ing,
}Dial_On_Off;

/**
  * @brief  发射模式
  * @param  单发和连发
  */
typedef enum
{
	Single_Shoot  = 0,
	Continuous_Shoot,
}Shoot_Modes;

/* 拨出一个弹丸，电机需要转动的角度，映射为360°-8192 */     
#define Angle_DialOneBullet_17mm 36864 //1024*36

/* Dial数据以及函数的接口结构体 */
#define Dial_DataGroundInit  \
    {                        \
		330,                 \
		0,                   \
		0,                   \
		-540,                \
		Single_Shoot,        \
		Dial_Off,            \
	}

#define Dial_FunGroundInit     \
    {                          \
		&Dial_Processing,      \
		&Dial_Update_Angel, \
		&PID_Clear_Dial,		\
	}		
		
typedef struct Dial_Data_t
{
	//uint8_t  Number_ToBeFired;   //需要打击的弹丸数量
	uint16_t Shoot_Interval;     //攻击间隔
	//uint16_t Shoot_Period;       //连发的时间
	uint32_t Time_NextShoot;     //下次射击的时间，即系统时间超过这个值才能射击
	//uint32_t Time_StopShoot;     //连发停止发射的时间
	uint16_t Bullet_Dialed;      //已经打出的子弹
	int16_t Speed_Dial;         //连发的拨盘电机速度
	
	Shoot_Modes Shoot_Mode;      //发射模式，单发还是连发
	Dial_On_Off Dial_Switch;     //拨弹电机的开关
}Dial_Data_t;

typedef struct Dial_Fun_t
{
	void (*Dial_Processing)();
	void (*Dial_Update_Angel)(bool Fric_ReadyOrNot);
	void (*PID_Clear_Dial)(void);

}Dial_Fun_t;

extern Dial_Fun_t Dial_Fun;
extern Dial_Data_t Dial_Data;

#endif /*__DIAL_H*/
