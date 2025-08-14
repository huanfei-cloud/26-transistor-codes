/**
 * @file BSP_BoardCommunication.h
 * @author lxr(784457420@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _BOARDCOMMUNICATION_H
#define _BOARDCOMMUNICATION_H

#include "main.h"
#include "bsp_fdcan.h"
#include "Protocol_Judgement.h"
#include "chassis.h"
#include "IMU.h"


// CAN报文的标识符和数据长度
#define CAN_ID_CHASSIS 0x10f // 假设CAN报文底盘数据ID为0x10f
#define CAN_ID_GIMBAL 0x11f	 // 云台数据ID为0x11f
#define CAN_ID_SABER   0x22f // 陀螺仪数据ID为0x22f
#define CAN_ID_SHOOT_HEAT 0x33f //裁判系统枪口热量数据ID为0x33f
#define CAN_ID_SABER_YAW 0x44f // 陀螺仪(Yaw)数据ID为0x22f
#define CAN_ID_CHASSIS_FUNCTION  0x55f  //功能（爬升、部署）
/*操作模式 0为正常，1为检录（左为mid，右为down）*/
#define model_Normal 0
#define model_Record 1

#define Board2_FunGroundInit    \
	{                           \
		&Board2_getChassisInfo, \
		&Board2_getGimbalInfo,  \
		&Board2_getChassisFunctionInfo,   \
		&Board2_To_1,           \
	}

// 定义CAN报文的结构体
typedef struct
{
	int16_t x_velocity;
	int16_t y_velocity;
	int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	int16_t pitch_encoder;
	int16_t yaw_velocity;
	uint8_t shoot_state;
	int16_t yaw_realAngle;
	uint8_t modelFlag;
	uint8_t shoot_Speed;
	uint8_t AutoAimFlag; // 自瞄开关
	uint8_t change_Flag; // 变速
	uint8_t fric_Flag;	 // 摩擦轮
	uint8_t tnndcolor;
	int16_t Speed_Bullet;
	int16_t chassis_yaw_angle;
	int16_t shooter_42mm_heat_limit;
	int16_t shooter_42mm_heat_now;
	int16_t yaw_position;
} ControlMessge;

extern ControlMessge ControlMes;

typedef struct
{
	void (*Board2_getChassisInfo)(uint8_t *RxMessage);
	void (*Board2_getGimbalInfo)(uint8_t *RxMessagee);
	void (*Board2_getChassisFunctionInfo)(uint8_t *RxMessage);
	void (*Board2_To_1)(void);
} Board2_FUN_t;

extern Board2_FUN_t Board2_FUN;

#endif
