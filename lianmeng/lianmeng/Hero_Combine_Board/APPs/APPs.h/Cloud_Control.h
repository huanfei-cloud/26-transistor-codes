/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief 
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright 
 * 
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H
#include "PID.h"
#include "kalman_filter.h"
#include "J4310_Motor.h"
#include "M6020_Motor.h"
#include "DT7.h"
#include "typedef.h"
//#include <AddMath.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

#define Cloud_Yaw_Center 1500
#define Cloud_Yaw_ReverseCenter 7809

//#define Cloud_Pitch_MIN = 100;
//#define Cloud_Pitch_MIN= -500;    //Pitch反向打
//#define Cloud_Pitch_MAX = 380;

#define Cloud_Pitch_level 0

/* 云台电机角度结构体 */
typedef struct
{
    float Yaw_Raw; 			//yaw轴原始数据
    float Pitch_Raw;   		//pitch轴原始数据
    float Target_Yaw; 		//云台目标yaw轴
    float Target_Pitch;   	//云台目标pitch轴
		float AutoAim_Pitch;      //yaw轴自瞄角
		float AutoAim_Yaw;        //pitch轴自瞄角
	float Vision_Yaw_Delta;		//视觉Yaw轴数据(差值)
	float Vision_Pitch_Delta;		//视觉Pitch轴数据(差值)
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
} Cloud_FUN_t;

void Cloud_Init(void);

extern int16_t Aim_Yaw_RawAngle;
extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;


#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
			&Cloud_Sport_Out,			  \
    }


#endif /* __CLOUD_CONTROL_H */
