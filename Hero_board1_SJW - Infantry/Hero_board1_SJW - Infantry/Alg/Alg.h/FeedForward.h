/**
 * @file FeedForward.h
 * @author Why
 * @brief 前馈控制
 * @version 0.1
 * @date 2023-08-28
 *
 * @copyright
 *
 */

#ifndef __FEEdFORWARD_H
#define __FEEdFORWARD_H

#include "main.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Cloud_Control.h"

#define Pitch_Margin 5

#define Yaw_J 1.0f	// kg*m2
#define Yaw_b 0.1f	// Nm/(s)
#define Yaw_K 0.7f	// Nm/A
#define Yaw_R 0.6f	// Ω
#define Yaw_L 0.02f // H

#define FeedForward_FunGroundInit                   \
	{                                               \
		&FeedForward_Fric,                          \
		&FeedForward_Chassis, /*&FeedForward_Pitch, \
							  &FeedForward_Yaw,  */ \
		&Compensator_Yaw,                           \
	}

typedef struct
{
	void (*FeedForward_Fric)(void);
	void (*FeedForward_Chassis)(void);
	//	void (*FeedForward_Pitch)(void);
	//	void (*FeedForward_Yaw)(void);
	void (*Compensator_Yaw)(void);
} FeedForward_FUN_t;

extern FeedForward_FUN_t FeedForward_FUN;
#endif
