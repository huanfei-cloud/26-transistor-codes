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

typedef struct
{
    float roll_targetlocation;  //roll轴目标位置
	  float roll_reallocation;    //roll轴真实位置
    
}Arm_Data_t;

/********全局变量声明********/
extern Arm_Data_t Arm_Data;

/********函数声明********/
void Arm_Motor_Init(void);
void Arm_Target_Calc(void);
void Arm_Out(void);

#endif
