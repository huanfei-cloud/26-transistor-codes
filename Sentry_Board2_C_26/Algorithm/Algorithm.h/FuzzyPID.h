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
#define FUZZYPID_Yaw_GroupInit           \
    {                                \
		0,                           \
		0,                           \
		0,                           \
		50,                       \
		-50,                      \
		15.f,                        \
		0.2f,                     \
		3.0,                        \
    }
#define FUZZYPID_AimYaw_GroupInit           \
    {                                \
		0,                           \
		0,                           \
		0,                           \
		50,                       \
		-50,                      \
		5.f,                        \
		0,                     \
		3.0,                        \
    }

//����ģ��PID�ṹ��
typedef struct
{
	float deta_kp; //����ֵ����
	float date_ki;  //����ֵ����
	float date_kd;  //΢��ֵ����

	float maximum; //���ֵ������
	float minimum;  //���ֵ������

	float qKp;    //kp����������ϵ��
	float qKi;      //ki����������ϵ��
	float qKd;    //kd����������ϵ��
}FUZZYPID_Data_t;

void FuzzyComputation (FUZZYPID_Data_t *vPID, float thisError, float lastError);

extern FUZZYPID_Data_t FuzzyPID_Yaw;
extern FUZZYPID_Data_t FuzzyPID_AimYaw;


#endif
