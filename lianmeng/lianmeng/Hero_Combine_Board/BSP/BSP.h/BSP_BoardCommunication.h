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

#ifndef BSP_BOARDCOMMUNICATION_H 
#define	BSP_BOARDCOMMUNICATION_H

#include "main.h"

#include "BSP_Fdcan.h"
#include "Extern_Handles.h"


// FDCAN报文的标识符和数据长度
#define FDCAN_ID_CHASSIS 0x10f // 假设CAN报文底盘数据ID为0x10f
#define FDCAN_ID_GIMBAL  0x11f // 云台数据ID为0x11f
#define FDCAN_ID_SABER   0x22f // 陀螺仪数据ID为0x22f
#define FDCAN_ID_SABER_YAW 0x44f // 陀螺仪(Yaw)数据ID为0x22f
#define FDCAN_ID_SHOOT_HEAT 0x33f //裁判系统枪口热量数据ID为0x33f
#define FDCAN_ID_CHASSIS_FUNCTION  0x55f  //功能（爬升、部署）

#define Board1_FunGroundInit   \
	{                          \
		&Board1_To_2,    	   \
		&Board1_getGimbalInfo, \
		&Board1_getSaberInfo,  \
		&Board1_getShootHeatInfo, \
	}

#define Board2_FunGroundInit   \
	{                          \
		&Board2_getChassisInfo,       \
		&Board2_getGimbalInfo,		  \
		&Board2_getChassisFunctionInfo,    \
		&Board2_To_1,                 \
	}


// 定义CAN报文的结构体
typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	int16_t yaw_velocity;
	int16_t yaw_position;        		//自瞄使用时，yaw轴应该在的绝对位置
	uint8_t AutoAimFlag;         		//自瞄开关
	uint8_t shoot_state;
	int16_t yaw_realAngle;       		//下板传上来的yaw轴角度信息
	float   Speed_Bullet;        		//裁判系统传来的弹速
	uint16_t shooter_42mm_heat_now;		//裁判系统传来的当前枪口热量
	uint16_t shooter_42mm_heat_limit;	//裁判系统传来的枪口热量上限
	uint16_t chassis_power_buffer;  	//裁判系统传回来的底盘功率缓冲 单位J
	uint16_t chassis_power_limit;   	//裁判系统传回来的底盘功率上限
	float chassis_power;		    	//裁判系统传回来的底盘输出功率   单位w
	uint8_t tnndcolor;                  //己方的颜色，1为红，2为蓝
	uint8_t fric_Flag;						//摩擦轮开关
	uint8_t Check_In_Flag;				//检录开关
	uint8_t Climb_Whole_Flag;           //爬升开关（整体）
	uint8_t Climb_Front_Flag;           //爬升开关（前轮）
	uint8_t Climb_Back_Flag;            //爬升开关（后轮）（爬升顺序：先整体，然后收前轮，之后收后轮）
	uint8_t Deployment_Flag;      //部署模式开关
	uint16_t Blood_Volume;         // ????
	uint8_t game_start;
} ControlMessge;

 typedef struct IMU_Angle_t
 {
	fp32 RoLL;
	fp32 Pitch;
	fp32 Yaw;
	 
	fp32 X_Vel;
	fp32 Y_Vel;
	fp32 Z_Vel;
	 
	fp32 X_Acc;
	fp32 Y_Acc;
	fp32 Z_Acc;
	 
	uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;
    uint32_t IMU_FPS;
	 
 }IMU_Angle_t;
extern IMU_Angle_t IMU_Angle;

extern ControlMessge ControlMes;

typedef struct
{
	void (*Board1_To_2)(void);
	void (*Board1_getGimbalInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board1_getSaberInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board1_getShootHeatInfo)(Fdcan_Export_Data_t RxMessage);
}Board1_FUN_t;

typedef struct
{
	void (*Board2_getChassisInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_getGimbalInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_getChassisFunctionInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_To_1)(void);
}Board2_FUN_t;


extern Board1_FUN_t Board1_FUN;
extern Board2_FUN_t Board2_FUN;

#endif
