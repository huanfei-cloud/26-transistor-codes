/**
 * @file shoot.h
 * @author xyz
 * @brief ��ɷ�������ĳ�ʼ�������岦���Ͳ�ͬ�ٷ���
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
	float r, s;	 // rת�ᵽ�������ĵľ��룬sΪȫ���ְ뾶
	float theta; // thetaΪ���� ��x��н�
	float omega; // omegaΪĿ��ת��
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
	} Speed_ToCloud; // ��������̨��������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
	struct
	{
		float vx;
		float vy;
		float w;
	} Speed_ToChassis; // �����ڵ��̳�������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
	float Angle_ChassisToCloud;
	float max_speed; // �����ƶ�����ٶ�
	motor_chassis chassis_motor[4];
};

extern void Chassis_Remote_Calculate(int16_t ch1, int16_t ch2, int16_t ch3, struct Struct_CHASSIS_Manage_Object *chassis);
extern void backward_calc(struct Struct_CHASSIS_Manage_Object *chassis);

extern struct Struct_CHASSIS_Manage_Object chassis_control;
extern void chassis_init(struct Struct_CHASSIS_Manage_Object *shoot);
extern void chassis_task(struct Struct_CHASSIS_Manage_Object *shoot);
#endif
