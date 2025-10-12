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

#include "DM_Motor.h"
#include "DJ_Motor.h"

typedef struct
{
	  float J3_targetlocation;  //小臂电机目标位置
	  float J3_reallocation;    //小臂电机真实位置
    float J4_targetlocation;  //大roll轴目标位置
	  float J4_reallocation;    //大roll轴真实位置
	  float J5_targetlocation;  //pitch轴目标位置
	  float J5_reallocation;    //pitch轴真实位置
	  float J6_targetlocation;  //小roll轴目标位置
		float J6_reallocation;    //小roll轴真实位置
    
}Arm_Data_t;

/********全局变量声明********/
extern Arm_Data_t Arm_Data;

/********函数声明********/
void Arm_Motor_Init(void);
void Arm_Motor_Set_Location(void);


#endif
