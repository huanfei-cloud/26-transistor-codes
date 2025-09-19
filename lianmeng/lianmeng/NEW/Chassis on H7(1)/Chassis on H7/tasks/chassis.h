/**
 * @file shoot.h
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#ifndef CHASSIS_H
#define CHASSIS_H

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "bsp_usart.h"
#include "pid.h"
#include "motor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "math.h"
#include "climb_stair.h"
#include "BoardCommunication.h"

#define r_chassis 0.23
#define s_chassis 0.06
#define pi 3.14159f

extern float up_limit[4];
extern float down_limit[4];
extern uint8_t climb_flag;
extern uint8_t last_climb_flag;

typedef struct
{
	float r, s;	 // r转轴到底盘中心的距离，s为全向轮半径
	float theta; // theta为轮面 对x轴夹角
	float omega; // omega为目标转速
} motor_chassis;

struct Struct_CHASSIS_Manage_Object
{
	struct Struct_MOTOR_Manage_Object motor1;
	struct Struct_MOTOR_Manage_Object motor2;
	struct Struct_MOTOR_Manage_Object motor3;
	struct Struct_MOTOR_Manage_Object motor4;
	struct Struct_MOTOR_Manage_Object joint_motor1;
	struct Struct_MOTOR_Manage_Object joint_motor2;
	struct Struct_MOTOR_Manage_Object joint_motor3;
	struct Struct_MOTOR_Manage_Object joint_motor4;

	struct
	{
		float vx;
		float vy;
		float w;
	} Speed_ToCloud; // 底盘在云台绝对坐标系下的速度变量（Vx,Vy,Vw）
	struct
	{
		float vx;
		float vy;
		float w;
	} Speed_ToChassis; // 底盘在底盘车体坐标系下的速度变量（Vx,Vy,Vw）
	float Angle_ChassisToCloud;
	float max_speed; // 底盘移动最大速度
	motor_chassis chassis_motor[4];
};

extern void Chassis_Remote_Calculate(int16_t ch1, int16_t ch2, int16_t ch3, struct Struct_CHASSIS_Manage_Object *chassis);
extern void backward_calc(struct Struct_CHASSIS_Manage_Object *chassis);

extern struct Struct_CHASSIS_Manage_Object chassis_control;
extern void chassis_init(struct Struct_CHASSIS_Manage_Object *shoot);
extern void chassis_task(struct Struct_CHASSIS_Manage_Object *shoot);
#endif
