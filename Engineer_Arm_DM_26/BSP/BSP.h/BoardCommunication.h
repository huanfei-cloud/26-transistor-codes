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
#define	_BOARDCOMMUNICATION_H

#include "main.h"
#include "bsp_fdcan.h"
#include "FreeRTOS.h"
#include "queue.h"

// CAN报文的标识符和数据长度
#define CAN_ID_CHASSIS 0x10f // 假设CAN报文底盘数据ID为0x10f
#define CAN_ID_GIMBAL  0x11f // 云台数据ID为0x11f

#define model_Normal 0
#define model_Record 1

#define Board1_FunGroundInit   \
	{                          \
		&Board1_To_2,    	   \
		&Board1_getGimbalInfo, \
	}


// 定义CAN报文的结构体
typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	int16_t yaw_velocity;
	int16_t yaw_position;        // 自瞄使用时，yaw轴应该在的绝对位置
	uint8_t AutoAimFlag;         // 自瞄开关
	uint8_t shoot_state;
	int16_t yaw_realAngle;       // 下板传上来的yaw轴角度信息
	float   Speed_Bullet;        // 裁判系统传来的弹速
	int16_t heat_remain;         // 裁判系统传来的剩余热量
	uint8_t modelFlag;					//比赛、检录模式	
	uint8_t shoot_Speed;				//射速
	uint8_t change_Flag;				//变速
	uint8_t fric_Flag;					//摩擦轮
	uint8_t tnndcolor;                  //己方颜色，1为红，2为蓝
	uint8_t redial;
} ControlMessge;

typedef struct
{
	void (*Board1_To_2)(void);
	void (*Board1_getGimbalInfo)(struct  Struct_CAN_Rx_Buffer RxMessage);
}Board1_FUN_t;

extern Board1_FUN_t Board1_FUN;
extern ControlMessge ControlMes;

#endif
