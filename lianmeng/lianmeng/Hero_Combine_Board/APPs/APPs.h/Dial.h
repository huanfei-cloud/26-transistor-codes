/**
 * @file Dial.h
 * @author Why
 * @brief �����̵���Ŀ�������
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __DIAL_H
#define __DIAL_H

#include "Shoot.h"
#include "main.h"
#include "cmsis_os.h"
 
/**
  * @brief  ��������Ŀ���
  * @param  ture��ʾ���Բ���
  */
typedef enum
{
	Dial_Off  = 0,
	Dial_On,
	//Dial_ing,
	//Dial_Off2,
	Dial_Back,
}Dial_On_Off;

/**
  * @brief  ����ģʽ
  * @param  ����������
  */
typedef enum
{
	Single_Shoot  = 0,
	Continuous_Shoot,
}Shoot_Modes;

/* ����һ�����裬�����Ҫת���ĽǶȣ�ӳ��Ϊ360��-8192360 */
#define Angle_DialOneBullet_42mm 31462.536898 //8192/6*19.2


/* Dial���������Լ������Ľӿڽṹ�� */
#define Dial_DataGroundInit  \
    {                        \
		0,                   \
		100,                 \
		5000,                \
		0,                   \
		0,                   \
		0,                   \
		-100,                \
		Single_Shoot,        \
		Dial_Off,            \
	}

#define Dial_FunGroundInit     \
    {                          \
		&Dial_Processing,      \
		&Dial_Update_Angel, \
		&Dial_Back_OneBullet,  \
		&Dial_OneBullet,		\
	}		
		
typedef struct Dial_Data_t
{
	uint8_t  Number_ToBeFired;   //��Ҫ����ĵ�������
	uint16_t Shoot_Interval;     //�������
	uint16_t Shoot_Period;       //������ʱ��
	uint32_t Time_NextShoot;     //�´������ʱ�䣬��ϵͳʱ�䳬�����ֵ�������
	uint32_t Time_StopShoot;     //����ֹͣ�����ʱ��
	uint16_t Bullet_Dialed;      //�Ѿ�������ӵ�
	uint16_t Speed_Dial;         //�����Ĳ��̵���ٶ�
	
	Shoot_Modes Shoot_Mode;      //����ģʽ��������������
	Dial_On_Off Dial_Switch;     //��������Ŀ���
}Dial_Data_t;

typedef struct Dial_Fun_t
{
	void (*Dial_Processing)();
	void (*Dial_Update_Angel)(bool Fric_ReadyOrNot);
	void (*Dial_Back_OneBullet)();
	void (*Dial_OneBullet)();
}Dial_Fun_t;

extern Dial_Fun_t Dial_Fun;
extern Dial_Data_t Dial_Data;

#endif /*__DIAL_H*/
