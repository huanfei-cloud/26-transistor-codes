/**
 * @file chassis.h
 * @author  重构代码
 * @brief 控制底盘运动
 * @version 1.1
 * @date 2025-04-02
 * @copyright Transistor BUAA
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "BSP_Can.h"
#include "BSP_Usart.h"
#include "Saber_C3.h"
#include "MA600_use.h"
#include "pid.h"
#include "DT7.h"
#include "M3508_Motor.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "arm_math.h"

#define r_chassis 0.23
#define s_chassis 0.06
#define pi 3.14159f

//底盘运动模式
typedef enum
{
		CHASSIS_ZERO_FORCE = 0, //无力模式
		CHASSIS_RC_GYROSCOPE = 1, //遥控器 小陀螺模式
	  CHASSIS_RC_FOLLOW_GIMBAL = 2, //遥控器 跟随云台模式
		CHASSIS_PC_CONTROL = 3, //PC 键鼠模式
		
}CHASSIS_MODE;

//底盘数据结构体
typedef struct
{
    struct
    {
        float vx;
        float vy;
        float wz;
    }Speed_ToCloud;

    struct
    {
        float vx;
        float vy;
        float wz;
    }Speed_ToChassis;

    float Angle_ChassisToCloud;
    float max_speed;
    float chassis_r;
    float chassis_s;
    int16_t motor_omega[4];
    float motor_location[4];
}Struct_CHASSIS_Manage_Object;

/*对外数据接口*/
extern Struct_CHASSIS_Manage_Object chassis_control;
extern fp64 Angle_Limit(fp64 angle,fp64 max);
extern void chassis_init(void);
extern void move_motor_speed_set(void);
extern void direction_motor_angle_set(void);
extern void v_cloud_convertto_chassis(fp32 angle);
extern void chassis_target_calc(void);
extern void steer_chassis_out(void);
#endif
