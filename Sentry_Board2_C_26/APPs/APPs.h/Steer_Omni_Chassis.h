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

/********��ֵ���ݶ���********/
#define M3508_RATIO 19
#define DIR_GEAR_RATIO 7.47f
#define Radius 60
//������ֵ������������ǰ��ʱת������ʼ����ֵ
#define DIRMOTOR_LB_ANGLE 4035
#define DIRMOTOR_RB_ANGLE 4035

#define Length_wheel_y 196.43f  //���ӵ����̼������ĵ�y����� (��λ��mm)
#define Length_wheel_x 155.49f  //���ӵ����̼������ĵ�x����� (��λ��mm)
#define WHEEL_PERIMETER 140.0f  //���ӵ��ܳ�
#define lf_omni_angle 135.0f    //��ǰ�������X��ĽǶ�
#define rf_omni_angle 45.0f     //��ǰ�������X��ĽǶ�

#define pi 3.1415926f

typedef struct
{
    struct
    {
        float vx;
        float vy;
        float wz;
    } Speed_ToCloud;            //��̨����ϵ���ٶ�vx,vy,wz
		
	struct
    {
        float vx;
        float vy;
        float wz;
    } Speed_ToChassis;          //��������ϵ���ٶ�vx,vy,wz
		
		fp32 Angle_ChassisToCloud;
    int32_t M6020_Setposition[2]; //����������������λ��
    int16_t M3508_Setspeed[4];	  //�ĸ���������������ٶ�
} Steer_Omni_Data_t;

/********ȫ�ֱ�������********/
extern Steer_Omni_Data_t Steer_Omni_Data;
extern follow_flag;
/********��������********/
fp64 Angle_Limit(fp64 angle,fp64 max);
void Chassis_Init(void);
void v_cloud_convertto_chassis(fp32 angle);
void direction_motor_angle_set(void);
void move_motor_speed_set(void);
void chassis_target_calc(void);
void Steer_Omni_Chassis_Out(void);
void Steer_Omni_GetAngle(fp32 angle);

#endif
