/**
 * @file Mecanum_Chassis.h
 * @author Lxr
 * @brief O型麦轮底盘控制
 * @version 0.1
 * @date 2023-08-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MECANUM_CHASSIS_H
#define __MECANUM_CHASSIS_H

#include "main.h"
#include "M3508_Motor.h"

#define MECANUM_LENGTH_A 513/2 	//底盘长的一半
#define MECANUM_LENGTH_B 372/2	//底盘宽的一半

#define MECANUM_WHEEL_PERIMETER 150//麦克纳姆轮直径，单位mm
#define Mecanum_DataGroundInit         \
				{                      \
						{0},           \
						{0},           \
						 0 ,           \
						{0},           \
				}
				
#define Mecanum_FunGroundInit                       \
				{                                   \
						&Mecanum_Chassis_out,   \
						&Mecanum_GetAngle, 			\
						&Mecanum_Chassis_Follow_Gimbal \
				}


typedef struct
{
	struct
	{
		float vx;
		float vy;
		float vw;
	} Speed_ToCloud; 	//底盘在云台绝对坐标系下的速度变量（Vx,Vy,Vw）
	
	struct
	{
		float vx;
		float vy;
		float vw;
	} Speed_ToChassis; 	//底盘在底盘车体坐标系下的速度变量（Vx,Vy,Vw）
	
	fp32 Angle_ChassisToCloud;
	int16_t M3508_Setspeed[4]; // 四个3508轮子目标转速
} Mecanum_Data_t;

typedef struct
{
	void (*Mecanum_Chassis_out)();
	void (*Mecanum_GetAngle)(fp32 angle);
	void (*Mecanum_Chassis_Follow_Gimbal)(void);
//	void (*RemoteControlChassis)(int16_t *speed);
} Mecanum_Fun_t;

extern Mecanum_Fun_t Mecanum_Fun;
extern Mecanum_Data_t Mecanum_Data;
extern incrementalpid_t M3508_Chassis_Pid[4];
extern positionpid_t Chassis_Follow_Pid;

#endif
