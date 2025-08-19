/**
 * @file Steer_Omni_Chassis.h
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-08-15
 * @copyright
 */

#ifndef _STEER_OMNI_CHASSIS_H
#define _STEER_OMNI_CHASSIS_H

#include "main.h"
#include "BSP_Can.h"
#include "PowerControl.h"
#include "Extern_Handles.h"  
#include "M3508_Motor.h"
#include "M6020_Motor.h"

/********常值数据定义********/
#define M3508_RATIO 19
#define DIR_GEAR_RATIO 7.47f
#define Radius 60
//后面舵轮的驱动电机朝向前方时转向电机初始编码值
#define DIRMOTOR_LB_ANGLE 4035
#define DIRMOTOR_RB_ANGLE 4035

#define Length_wheel_y 196.43f  //轮子到底盘几何中心的y轴距离 (单位：mm)
#define Length_wheel_x 155.49f  //轮子到底盘几何中心的x轴距离 (单位：mm)
#define WHEEL_PERIMETER 140.0f  //轮子的周长
#define lf_omni_angle 135.0f    //左前轮相对于X轴的角度
#define rf_omni_angle 45.0f     //右前轮相对于X轴的角度

#define pi 3.1415926f

typedef struct
{
    struct
    {
        float vx;
        float vy;
        float wz;
    } Speed_ToCloud;            //云台坐标系的速度vx,vy,wz
		
	struct
    {
        float vx;
        float vy;
        float wz;
    } Speed_ToChassis;          //底盘坐标系的速度vx,vy,wz
		
		fp32 Angle_ChassisToCloud;
    int32_t M6020_Setposition[2]; //两个舵向电机的设置位置
    int16_t M3508_Setspeed[4];	  //四个驱动电机的设置速度
} Steer_Omni_Data_t;

/********全局变量声明********/
extern Steer_Omni_Data_t Steer_Omni_Data;

/********函数声明********/
fp64 Angle_Limit(fp64 angle,fp64 max);
void chassis_out(void);
void v_cloud_convertto_chassis(fp32 angle);
void direction_motor_angle_set(void);
void move_motor_speed_set(void);
void chassis_target_calc(void);
void Steer_Omni_Chassis_Out(void);
void Steer_Omni_GetAngle(fp32 angle);

#endif
