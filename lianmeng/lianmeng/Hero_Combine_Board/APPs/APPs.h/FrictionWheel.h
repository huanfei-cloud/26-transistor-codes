/**
 * @file FrictionWheel.h
 * @author Why
 * @brief 处理摩擦轮的控制问题
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __FRICTIONWHEEL_H
#define __FRICTIONWHEEL_H

#include "Shoot.h"
#include "main.h"

 /**
  * @brief  摩擦轮是否达到设定值
  * @param  ture表示可以达到
  */
typedef enum
{
	Fric_Ready  = true,
	Fric_NotReady = false,
}Fric_ReadyOrNot;

/**
  * @brief  摩擦轮的开关
  * @param  ture表示开始
  */
typedef enum
{
	Fric_On  = true,
	Fric_Off = false,
}Fric_On_Off;
 
 /* 根据射速得到的摩擦轮转速 */
//#define Fric_SpeedLevel1 5500


/* Dial函数数据以及函数的接口结构体 */
#define Fric_DataGroundInit   \
    {                         \
		0,                    \
		Fric_NotReady,        \
		Fric_Off,             \
	}

#define Fric_FunGroundInit      \
    {                           \
		&Fric_Processing,       \
		&Fric_Judge_ReadyOrNot, \
	}		
		
typedef struct Fric_Data_t
{
	fp32 Required_Speed;    //摩擦轮需要的速度
	
	bool Fric_Ready;        //摩擦轮加速完成
	bool Fric_Switch;       //摩擦轮开关
}Fric_Data_t;

typedef struct Fric_Fun_t
{
	void (*Fric_Processing)();
	void (*Fric_Judge_ReadyOrNot)();
}Fric_Fun_t;

extern Fric_Fun_t Fric_Fun;
extern Fric_Data_t Fric_Data;

#endif /*__FRICTIONWHEEL_H*/
