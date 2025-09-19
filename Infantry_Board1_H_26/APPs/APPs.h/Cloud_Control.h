/**
 * @file Cloud_control.c
 * @author Cyx,SJW
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
#include "J4310_Motor.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


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
    void (*Cloud_Init)(void);//云台初始化函数指针
    void (*Cloud_Sport_Out)(void);// 云台运动输出函数指针
	  void (*Cloud_Pitch_Angle_Set)(void);// Pitch轴角度调整函数指针
	  void (*Remote_Change)(void); // 遥控器信号变化处理函数指针

} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;

#define Cloud_FUNGroundInit { &Cloud_Init, &Cloud_Sport_Out,	&Cloud_Pitch_Angle_Set, &Remote_Change}


#endif /* __CLOUD_CONTROL_H */
