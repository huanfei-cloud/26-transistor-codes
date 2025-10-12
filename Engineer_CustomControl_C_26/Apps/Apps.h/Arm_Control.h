/**
 * @file Arm_Control.h
 * @author xhf
 * @brief 机械臂控制
 * @version 1.0
 * @date 2025-08-21
 * @copyright Transistor BUAA
 */

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "DJ_Motor.h"
#include "MA600_use.h"
#include "N100.h"

typedef struct
{
    float J1_targetlocation;  //J1电机目标位置
	  float J2_targetlocation;  //J2电机目标位置
	  float J3_targetlocation;  //J3电机目标位置
	  float J4_targetlocation;  //J4电机目标位置
	  float J5_targetlocation;  //J5电机目标位置
	  float J6_targetlocation;  //J6电机目标位置
    
}CustomControl_Data_t;

/********全局变量声明********/
extern CustomControl_Data_t CustomControl_Data;

/********函数声明********/
void CustomControl_Motor_Init(void);
void CustomControl_Set_Location(void);
#endif
