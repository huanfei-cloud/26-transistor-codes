#ifndef __Omni_Chassis_H
#define __Omni_Chassis_H
#include "main.h"
#include "M2006_Motor.h"

#define OMNI_LENGTH_A 110/2 //底盘长的一半
#define OMNI_LENGTH_B 110/2 //底盘宽的一半

#define OMNI_WHEEL_PERIMETER 80//全向轮直径，单位mm
//#define CHASSIS_DECELE_RATIO 36//2006电动机转速比，官方手册为36

#define Omni_DataGroundInit         \
				{                      \
						{0},           \
						{0},           \
						 0 ,           \
						{0},           \
				}
				
#define Omni_FunGroundInit                       \
				{                                   \
						&Omni_Chassis_Out,   \
						&Omni_GetAngle, 			\
						&RemoteControlChassis,      \
				}

typedef struct
{
    struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToCloud;            //底盘在云台绝对坐标系下的速度变量（Vx,Vy,Vw）
		
	struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToChassis;          //底盘在底盘车体坐标系下的速度变量（Vx,Vy,Vw）
		
		fp32 Angle_ChassisToCloud;
		int16_t M2006_Setspeed[4];	 //四个2006轮子目标转速
} Omni_Data_t;

typedef struct
{
	void (*Omni_Chassis_Out)();
	void (*Omni_GetAngle)(fp32 angle);
	void (*RemoteControlChassis)(int16_t *speed);
}Omni_Fun_t;

extern Omni_Fun_t Omni_Fun;

#endif
