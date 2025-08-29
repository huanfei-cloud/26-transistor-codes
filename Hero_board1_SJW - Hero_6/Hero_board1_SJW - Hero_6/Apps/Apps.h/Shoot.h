/**
 * @file Shoot.h
 * @author Why
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __SHOOT_H
#define __SHOOT_H

#include "M3508_Motor.h"
#include "FrictionWheel.h"
#include "Dial.h"
#include "main.h"
#include "BSP_Can.h"
#include "Cloud_Control.h"
#include "Extern_Handles.h"
#include "BSP_BoardCommunication.h"

#define Heat_17mm 10

/**
 * @brief  发射器工作状态
 * @param
 */
typedef enum
{
	Shoot_Off = 0,
	Shoot_On = 1,
	Shoot_Single = 2,
} Shoot_On_Off;

#define Shoot_DataGroundInit { \
	0,                         \
	0,                         \
	Shoot_Off,                 \
}

#define Shoot_FunGroundInit \
	{                       \
		&Shoot_Processing,  \
	}

typedef struct Shoot_Data_t
{
	uint16_t Heat_Now;	 // 当前热量
	uint16_t Heat_Limit; // 热量限制
	uint16_t Shoot_Switch;	 // 发射开关
} Shoot_Data_t;

typedef struct Shoot_Fun_t
{
	void (*Shoot_Processing)();
} Shoot_Fun_t;

extern Shoot_Fun_t Shoot_Fun;
extern Shoot_Data_t Shoot_Data;

extern incrementalpid_t M3508_FricF1_Pid;
extern incrementalpid_t M3508_FricF2_Pid;
extern incrementalpid_t M3508_FricF3_Pid;
extern incrementalpid_t M3508_FricB1_Pid;
extern incrementalpid_t M3508_FricB2_Pid;
extern incrementalpid_t M3508_FricB3_Pid;

extern positionpid_t M3508_DialV_Pid;
extern incrementalpid_t M3508_DialI_Pid;

#endif /*__SHOOT_H*/
