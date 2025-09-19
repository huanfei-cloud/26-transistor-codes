/**
 * @file FuzzyPID.h
 * @author Why/xyz
 * @brief ����ĺ�������뷨��д��PID�ļ����ڲ�ȥ��ʵ��Fuzzy���ߵ���PID����
 *		  ��Ҫ������ֻ��PID�����ı䷶Χ
 * @version 0.1
 * @date 2024-10-21
 *
 */
#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H
// ��ʼ���ṹ�����
#include "stm32f4xx_hal.h"

#define FUZZYPID_Dial_GroupInit \
	{                           \
		0,                      \
		0,                      \
		0,                      \
		36865,                  \
		-36865,                 \
		0.8f,                   \
		0.0005f,                \
		0.03,                   \
	}

// ����ģ��PID�ṹ��
typedef struct
{
	float delta_kp; // ����ֵ��������
	float delta_ki; // ����ֵ��������
	float delta_kd; // ΢��ֵ��������

	float error_maximum; // ���ֵ���������
	float error_minimum; // ���ֵ���������

	float qKp; // kp������������Χ
	float qKi; // ki������������Χ
	float qKd; // kd������������Χ

	float error_map[2]; // error/d_errorģ�����õ���ֵ

	float error_membership_degree[2]; // errorģ����������
	int8_t error_membership_index[2]; // errorģ��������

	float d_error_membership_degree[2]; // d_errorģ����������
	int8_t d_error_membership_index[2]; // d_errorģ��������

} FUZZYPID_Data_t;
extern void Linear_Quantization(FUZZYPID_Data_t *PID, float thisError, float lastError, float *qValue);
extern void Membership_Calc(float *ms, float qv, int8_t *index);
extern void FuzzyComputation(FUZZYPID_Data_t *PID, float thisError, float lastError);
extern void fuzzy_init(FUZZYPID_Data_t *PID, float _maximum, float _minimum, float _qkp, float _qki, float _qkd);

extern FUZZYPID_Data_t FuzzyPID_Dial;

extern FUZZYPID_Data_t fuzzy_pid_shoot_F1;
extern FUZZYPID_Data_t fuzzy_pid_shoot_F2;
extern FUZZYPID_Data_t fuzzy_pid_shoot_F3;
extern FUZZYPID_Data_t fuzzy_pid_shoot_B1;
extern FUZZYPID_Data_t fuzzy_pid_shoot_B2;
extern FUZZYPID_Data_t fuzzy_pid_shoot_B3;
extern FUZZYPID_Data_t fuzzy_pid_bullet_v;
extern FUZZYPID_Data_t fuzzy_pid_bullet_l;
extern FUZZYPID_Data_t fuzzy_pid_pitch_in;
extern FUZZYPID_Data_t fuzzy_pid_pitch_out;
#endif
