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
#include "FuzzyPID.h"
#include "math.h"

/**********PID�������ݽӿ�************/
typedef struct incrementalpid_t
{
    float Target;         //�趨Ŀ��ֵ
    float Measured;       //����ֵ
    float err;            //����ƫ��ֵ
    float err_last;       //��һ��ƫ��
    float err_beforeLast; //���ϴ�ƫ��
	  float last_set_point; // ��һ��Ŀ��ֵ
	  float error_target;   // ǰ������
	  float integral_error; // �������ۻ�
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
	  float kf;
    float p_out;
    float i_out;
    float d_out;            //���������ֵ
	  float f_out;  // ǰ��f���
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
	  float last_set_point; // ��һ��Ŀ��ֵ
	  float error_target;   // ǰ������
	  float integral_error; // �������ۻ�
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
	  float kf;
    float p_out;
    float i_out;
    float d_out;               //���������ֵ
	float f_out;
    float pwm;                 //pwm���
    float MaxOutput;           //����޷�
    float Integral_Separation; //���ַ�����ֵ
    float IntegralLimit;       //�����޷�
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;

//ģ��PID����Ļ�׼����
typedef struct Struct_PID_Manage_Object_Fuzzy
{
   float Kp;
   float Ki;
   float Kd;
   float Kf;
	 float Kc;   //????

   float error;
   float last_error;
   float before_last_error;
   float integral_error; // �������ۻ�
   float error_target;   // ǰ������
	 float err_change; //���仯��

   float Target;      // Ŀ��ֵ
   float Measured;      // ��ǰֵ
   float last_set_point; // ��һ��Ŀ��ֵ

   float integral_limit; // �����޷�
   float output_limit;   // ����޷�
	 float Integral_Separation; //���ַ�����ֵ
   float deadzone;       // ����

   float p_out;  // p���
   float i_out;  // i���
   float d_out;  // d���
   float f_out;  // ǰ��f���
	 float feedforward_CONSTANT_out;
   float output; // �����
}Struct_PID_Manage_Object_Fuzzy;

extern Struct_PID_Manage_Object_Fuzzy M6020s_YawIPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_PitchIPID;		//Pitch	
extern Struct_PID_Manage_Object_Fuzzy M6020s_Yaw_SpeedPID;	//Yaw�ٶ�PID
extern Struct_PID_Manage_Object_Fuzzy M6020s_YawOPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_PitchOPID;		//Pitch	
extern Struct_PID_Manage_Object_Fuzzy M6020s_AimYawOPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_AimYawIPID;		//Yaw

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
extern float Position_PID(positionpid_t *pid_t, float target, float measured);
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki,float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);

extern float PID_Model4_Update(Struct_PID_Manage_Object_Fuzzy *pid,FUZZYPID_Data_t *PID, float _set_point, float _now_point);
extern void PID_Model4_Update_PIDInit(Struct_PID_Manage_Object_Fuzzy *pid_t, float Kp, float Ki, float Kd,float Kf,float Kc, uint32_t MaxOutput, uint32_t IntegralLimit, uint32_t Integral_Separation);
extern float Position_PID_Dial(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

extern One_Kalman_t Cloud_YAWODKalman;
extern One_Kalman_t Cloud_PITCHODKalman;

extern float Cloud_IMUPITCHIPID(positionpid_t *pid_t, float target, float measured);
extern float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUYAWOPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUYAWIPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUPITCHOPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUPITCHIPID(positionpid_t *pid_t, float target, float measured);
extern void Clear_PositionPIDData(positionpid_t *pid_t);
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);
extern float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured);

#endif
