/**
 * @file Task_Arm.h
 * @author Transistor
 * @brief 工程机械臂的任务函数
 * @version 1.2
 * @date 2025-08-15
 * @copyright Transistor BUAA
 */

#ifndef _TASK_ARM_H
#define _TASK_ARM_H

#include "FreeRTOS.h"
#include "Arm_Control.h"

/********函数声明********/
void Arm_Control(void const *argument);

#endif
