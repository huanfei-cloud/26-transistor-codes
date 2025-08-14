#ifndef __Omni_Chassis_H
#define __Omni_Chassis_H
#include "main.h"
#include "M3508_Motor.h"

#define Length_steer_y 198.43f    // 轮子到底盘几何中心的y轴距离 (单位：mm)
#define Length_steer_x 155.49f    // 轮子到底盘几何中心的x轴距离 (单位：mm)
#define OMNI_WHEEL_PERIMETER 140//ȫ����ֱ������λmm
#define lf_steer_angle 135.0f // 左前轮相对于x轴的角度 (单位：度)
#define rf_steer_angle 45.0f // 右前轮相对于x轴的角度 (单位：度)
#define lb_steer_angle 225.0f // 左后轮相对于x轴的角度 (单位：度)
#define rb_steer_angle 315.0f // 右后轮相对于x轴的角度 (单位：度)
#define pi 3.14159265358f
//#define CHASSIS_DECELE_RATIO 36//2006�綯��ת�ٱȣ��ٷ��ֲ�Ϊ36

#define Omni_DataGroundInit         \
				{                      \
						{0},           \
						{0},           \
						 0 ,           \
						{0},           \
				}
				
#define Omni_FunGroundInit                       \
				{                                   \
						&Omni_Chassis_Out,   \
						&Omni_GetAngle, 			\
						&RemoteControlChassis,      \
				}

typedef struct
{
    struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToCloud;            //��������̨��������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
		
	struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToChassis;          //�����ڵ��̳�������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
		
		fp32 Angle_ChassisToCloud;
		int16_t M2006_Setspeed[4];	 //�ĸ�2006����Ŀ��ת��
} Omni_Data_t;

typedef struct
{
	void (*Omni_Chassis_Out)();
	void (*Omni_GetAngle)(fp32 angle);
	void (*RemoteControlChassis)(int16_t *speed);
}Omni_Fun_t;

extern Omni_Fun_t Omni_Fun;
extern Omni_Data_t Omni_Data;
extern incrementalpid_t M3508_Array_Pid[4];		//�ĸ�2006����PID�ṹ��	

#endif
