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

/******关键设定：pid跟3508电机数组的前四个是一一对应的四个轮子
*******这会在其他地方用到************************************/

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

#define PowerControl_FunGroundInit           \
    {                              		\
		&PowerControl_Handle,           \
    }

#define PowerControl_DataGroundInit \
    {                      \
		0,                 \
		0,                 \
		0,                 \
    }

typedef struct
{
	/* 计算出的值 */
	float OverLimit_Ratio;         // 超过功率的比例
	
	/* 裁判系统的值 */
	uint16_t Power_Limit;          // 限制功率
	float Power_Observe_ByJudge;   // 裁判系统测量出的功率值
} PowerControl_Data_t;

typedef struct
{
    void (*PowerControl_Handle)(void);
} PowerControl_Fun_t;

extern PowerControl_Fun_t PowerControl_Fun;
extern PowerControl_Data_t PowerControl_Data;

#endif /*__POWERCONTROL_H*/
