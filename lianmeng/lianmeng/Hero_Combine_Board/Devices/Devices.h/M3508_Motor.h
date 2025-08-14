/**
 * @file M3508_Motor.h
 * @author Why
 * @brief 
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "PID.h"
#include "fdcan.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>

/* ��¼M3508�������ID
*/
#define M3508_READID_CHASSIS_START 0x201
#define M3508_READID_CHASSIS_END 0x204
#define M3508_READID_SHOOT_START 0x205
#define M3508_READID_SHOOT_END 0x207
#define M3508_SENDID_Chassis 0x200   //���Ƶ��̵��
#define M3508_SENDID_Fric_Dial 0x1FF //����Ħ���ֺͲ��̵��
#define M3508_MaxOutput 16384        //���͸������������ֵ
#define M3508_CurrentRatio 819.2f    //16384/20A = 819.2->1A
#define M3508_ReductionRatio 3591/187 //3508������ٱ�

#define M3508_FunGroundInit          \
    {                                \
        &M3508_getInfo,              \
		&M3508_setCurrent,           \
		&Check_WheelM3508,   	     \
    }

/******�ؼ��趨��pid��3508��������ǰ�ĸ���һһ��Ӧ���ĸ�����
*******����������ط��õ�************************************/
	
	/**
  * @brief  ��������3508����������Ԫ�صĹ���
  * @param  Chassis_Right_Front	��ǰ��
  *			Chassis_Left_Front  ��ǰ��
  *			Chassis_Left_Back	�����
  *			Chassis_Right_Back	�Һ���
  *			FricL_Wheel			��Ħ����
  *			FricR_Wheel			��Ħ����
	*			FricU_Wheel			��Ħ����
  *			DialMotor			�������
  */
typedef enum
{
	FricL_Wheel=0,
	FricR_Wheel,
	FricU_Wheel,
	Dial_Motor,

}M3508_MotorName;

typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ�ʵ���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed;  //Ŀ���ٶ�
    int64_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;   //�ϴεĽǶ�
    int64_t totalAngle;   //�ۻ��ܹ��Ƕ�
    int16_t turnCount;    //ת����Ȧ��

    int16_t outCurrent;   //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M3508s_t;

extern M3508s_t M3508_Array[8];

typedef struct
{
    void (*M3508_getInfo)(Fdcan_Export_Data_t RxMessage);
	void (*M3508_setCurrent)(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);
	void (*Check_WheelM3508)(void);
} M3508_FUN_t;

extern M3508_FUN_t M3508_FUN;        

#endif /*__M3508_MOTOR_H*/
