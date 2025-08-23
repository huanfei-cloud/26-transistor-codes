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
#include "shoot.h"
#include "DT7.h"
#include "typedef.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。
/****************Pitch限位*****************/
#define Cloud_Pitch_level 3614
#define Cloud_Pitch_Init 4280
#define Cloud_Pitch_Min 2900
#define Cloud_Pitch_Max 4280
/****************Pithch限位End*****************/


/* 云台电机角度结构体 */
typedef struct
{
    float Yaw_Raw;         //yaw轴原始数据
    float Pitch_Raw;       //pitch轴原始数据
    float Target_Yaw;      //云台目标yaw轴
    float Target_Pitch;    //云台目标pitch轴
	float AutoAim_Pitch;   //自瞄得到的pithc轴角度
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
    void (*PID_Clear_Pitch)(void);
		void (*Cloud_Pitch_Angle_Set)(void);
		void (*Remote_Change)(void);

} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;

#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
		&Cloud_Sport_Out,			 	  						\
			&PID_Clear_Pitch,										\
			&Cloud_Pitch_Angle_Set,  	\
			&Remote_Change,				\
    }


#endif /* __CLOUD_CONTROL_H */
