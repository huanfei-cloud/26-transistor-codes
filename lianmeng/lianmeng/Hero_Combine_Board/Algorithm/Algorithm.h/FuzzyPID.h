/**
 * @file FuzzyPID.c
 * @author Why
 * @brief 
 * @version 0.1
 * @date 2023-09-28
 * 
 */
#ifndef __FUZZYPID_H
#define __FUZZYPID_H

//��ʼ���ṹ�����
#include "stm32h7xx_hal.h"

#define FUZZYPID_Dial_GroupInit           \
    {                                \
		0,                           \
		0,                           \
		0,                           \
		36865,                       \
		-36865,                      \
		0.8f,                        \
		0.0005f,                     \
		0.03,                        \
    }

//����ģ��PID�ṹ��
typedef struct
{
	float delta_kp; //����ֵ��������
	float delta_ki;  //����ֵ��������
	float delta_kd;  //΢��ֵ��������

	float error_maximum; //���ֵ���������
	float error_minimum;  //���ֵ���������

	float qKp;    //kp������������Χ
	float qKi;      //ki������������Χ
	float qKd;    //kd������������Χ
	
	float error_map[2];//error/d_errorģ�����õ���ֵ
	
	
	float error_membership_degree[2];//errorģ����������
	int8_t error_membership_index[2];//errorģ��������
	
	float d_error_membership_degree[2];//d_errorģ����������
	int8_t d_error_membership_index[2];//d_errorģ��������
}FUZZYPID_Data_t;

void FuzzyComputation (FUZZYPID_Data_t *vPID, float thisError, float lastError);

extern void Linear_Quantization(FUZZYPID_Data_t *PID, float thisError, float lastError, float *qValue);
extern void Membership_Calc(float *ms, float qv, int8_t *index);
extern void FuzzyComputation (FUZZYPID_Data_t *PID, float thisError, float lastError);
extern void fuzzy_init(FUZZYPID_Data_t *PID,float _maximum,float _minimum,float _qkp,float _qki,float _qkd);

extern FUZZYPID_Data_t FuzzyPID_Dial;

extern FUZZYPID_Data_t fuzzy_pid_fric_l;
extern FUZZYPID_Data_t fuzzy_pid_fric_r;
extern FUZZYPID_Data_t fuzzy_pid_fric_u;
extern FUZZYPID_Data_t fuzzy_pid_dial_v;
extern FUZZYPID_Data_t fuzzy_pid_dial_l;
extern FUZZYPID_Data_t M6020s_YawO_FuzzyPID;
extern FUZZYPID_Data_t M6020s_YawI_FuzzyPID;
extern FUZZYPID_Data_t M6020s_AimYawO_FuzzyPID;
extern FUZZYPID_Data_t M6020s_AimYawI_FuzzyPID;
#endif
