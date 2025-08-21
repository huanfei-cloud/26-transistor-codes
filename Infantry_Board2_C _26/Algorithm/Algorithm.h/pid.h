/**
 * @file PID.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef ___PID_H
#define ___PID_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kalman_filter.h"
#include "math.h"
#include "FuzzyPID.h"
//#include "typedef.h"
//#include <AddMath.h>

/********pid�Ĳ���ֵ********/
//����yaw��λ�û�
#define yaw_O_p 0.083f
#define yaw_O_i 0.00001f
#define yaw_O_d 0.0f
//����yaw���ٶȻ�
#define yaw_I_p 1050.0f
#define yaw_I_i 0.12f
#define yaw_I_d 1000.0f
//����yaw��λ�û�
#define yaw_O_Aim_p 2.082f
#define yaw_O_Aim_i 0.00001f
#define yaw_O_Aim_d 1.0f
#define yaw_O_Aim_f 0.0f
//����yaw���ٶȻ�
#define yaw_I_Aim_p 1100.0f
#define yaw_I_Aim_i 60.0f
#define yaw_I_Aim_d 950.0f
#define yaw_I_Aim_f 100.0f

/**********PID�������ݽӿ�************/

// pid���Ʒ�ʽѡ��
typedef enum
{
	pid_control_increase,
	pid_control_normal,
	pid_control_frontfeed,
	pid_control_frontfuzzy
}pid_control;


typedef struct incrementalpid_t
{
    float Target;         //�趨Ŀ��ֵ
    float Measured;       //����ֵ
    float err;            //����ƫ��ֵ
    float err_last;       //��һ��ƫ��
    float err_beforeLast; //���ϴ�ƫ��
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
    float p_out;
    float i_out;
    float d_out;            //���������ֵ
    float pwm;              //pwm���
    uint32_t MaxOutput;     //����޷�
    uint32_t IntegralLimit; //�����޷�
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;

typedef struct positionpid_t
{
    float Target;     //�趨Ŀ��ֵ
    float Measured;   //����ֵ
    float err;        //����ƫ��ֵ
    float err_last;   //��һ��ƫ��
    float err_change; //���仯��
	  float error_target;   // ǰ������
		float last_set_point; //��һ��Ŀ��ֵ
    float Kp;
    float Ki;
    float Kd; 
	  float Kf;         //Kp, Ki, Kd, Kf����ϵ��
    float p_out;
    float i_out;
    float d_out;               
	  float f_out;               //���������ֵ
    float pwm;                 //pwm���
    float MaxOutput;           //����޷�
    float Integral_Separation; //���ַ�����ֵ
    float IntegralLimit;       //�����޷�
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;

extern positionpid_t M6020s_YawIPID;		//Yaw	
extern positionpid_t M6020s_Yaw_SpeedPID;	//Yaw�ٶ�PID
extern positionpid_t M6020s_YawOPID;		//Yaw
extern positionpid_t AutoAim_M6020s_YawIPID;
extern positionpid_t AutoAim_M6020s_YawOPID;   //Yaw����PID

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
extern float Position_PID(positionpid_t *pid_t, float target, float measured);
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max);
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki, float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);

extern One_Kalman_t Cloud_YAWODKalman;
extern One_Kalman_t Cloud_PITCHODKalman;

extern void Clear_PositionPIDData(positionpid_t *pid_t);
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

extern float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

#endif
