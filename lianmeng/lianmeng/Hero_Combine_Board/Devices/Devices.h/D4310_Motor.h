/**
 * @file D4310_Motor.h
 * @author ZS
 * @brief 
 * @version 0.1
 * @date 2024-12-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __J4310_MOTOR_H
#define __J4310_MOTOR_H

#include "fdcan.h"
#include "main.h"
#include "typedef.h"
#include "Task_FdcanReceive.h"
#include "PID.h"

#define J4310_READID_START 0x01
#define J4310_READID_END 0x01
#define J4310_MaxV 200 //���͸���������ת��,��λrpm
#define J4310_MaxT 7 //���͸���������Ť�أ���λNM
#define J4310_ReductionRatio 10 //������ٱ�
//#define M2006_LOADANGLE		42125			/* �����һ������Ҫת�ĽǶ���  6*8191 ��7�ײ�����*/

//#define M2006_LOADCIRCLE	5			/* �����һ������Ҫת��Ȧ�� */
//#define M2006_LOADSPEED		1800		/* �������ʱ��ת�� */
#define J4310_FIRSTANGLE 3800 /* �����ʼλ�� */

#define M6020_mAngleRatio 22.7527f //��е�Ƕ�����ʵ�Ƕȵı���

#define M6020_getRoundAngle(rAngle) rAngle / M6020_mAngleRatio //��е�Ƕ�����ʵ�Ƕȵı���

#define M6020_FunGroundInit        \
    {                              \
		&M6020_setVoltage,		   \
			&M6020_getInfo,		   \
			&M6020_setTargetAngle, \
			&M6020_Reset,          \
			&Check_M6020,		   \
    }

typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int32_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ��ת�ص���
    uint8_t temperture;  //�������ĵ���¶�
    uint16_t lastAngle;  //�ϴεĽǶ�
	
    int32_t targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�
	
    int16_t turnCount;   //ת����Ȧ��
    float totalAngle;  //�ۻ��ܹ��Ƕ�

    int16_t outCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M6020s_t;

typedef enum
{
    //��Ҫע���뱨�Ľ��ܺ�������Ӧ����
    //M6020_PITCH_Right = 0,
    M6020_PITCH = 0,
    M6020_YAW,
} M6020Name_e;

typedef struct
{
	void (*M6020_setVoltage)(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
    void (*M6020_getInfo)(Fdcan_Export_Data_t RxMessage);
    void (*M6020_setTargetAngle)(M6020s_t *M6020, int32_t angle);
    void (*M6020_Reset)(M6020s_t *m6020);
	void (*Check_M6020)(void);
} M6020_Fun_t;

extern M6020s_t M6020s_Yaw;   //IDΪ1
extern M6020s_t M6020s_Pitch; //2
extern M6020_Fun_t M6020_Fun;

#endif /* __M3508_MOTOR_H */
