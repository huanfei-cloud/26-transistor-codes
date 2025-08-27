/**
 * @file Arm_Control.c
 * @author Transistor
 * @brief 工程机械臂的控制程序
 * @version 1.2
 * @date 2025-08-15
 * @copyright Transistor BUAA
 */

#ifndef _ARM_CONTROL_H
#define _ARM_CONTROL_H

#include "math.h"
#include "cmsis_os.h"
#include "motor.h"
#include "DM_Motor_Drv.h"
#include "DM_Motor_Ctrl.h"

/********常数定义********/
#define pi 3.1415926f

/********全局变量声明********/
extern struct Struct_MOTOR_Manage_Object joint_motor4; // 6020
extern struct Struct_MOTOR_Manage_Object joint_motor5; // 2006
extern struct Struct_MOTOR_Manage_Object joint_motor6; // 2006
extern float arm_angle[6];
extern float last_arm_angle[6];

/********函数声明********/
void arm_init(void);
void arm_control(void const *argument);
void dm_ctrl(motor_t *motor, float pos);
void arm_angle_update(uint16_t *angle);
void get_dm_zero(void);
void DM_Motors_Out(void);
void DJ_Motors_Out(void);

#endif
