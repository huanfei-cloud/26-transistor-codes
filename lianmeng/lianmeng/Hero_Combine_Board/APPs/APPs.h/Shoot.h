/**
 * @file Shoot.h
 * @author Why
 * @brief 
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __SHOOT_H
#define __SHOOT_H

#include "M3508_Motor.h"
#include "FrictionWheel.h"
#include "Dial.h"
#include "main.h"

/**
  * @brief  ��������Ŀ���
  * @param  ture��ʾ�������
  */
typedef enum
{
	Shoot_On  = true,
	Shoot_Off = false,
}Shoot_On_Off;
 
/* Shoot���������Լ������Ľӿڽṹ�� */
#define Shoot_DataGroundInit  \
    {                         \
		  0,                    \
		  0,                    \
			                    \
		  Shoot_Off,            \
	}

#define Shoot_FunGroundInit   \
    {                         \
		&Shoot_Processing,    \
	}		
		
typedef struct Shoot_Data_t
{
	uint16_t Heat_Now;               //���ڵ�����
	uint16_t Heat_Limit;             //�������Ƶ���������
	
	bool Shoot_Switch;               //�Ƿ������
}Shoot_Data_t;

typedef struct Shoot_Fun_t
{
	void (*Shoot_Processing)();
}Shoot_Fun_t;

extern Shoot_Fun_t Shoot_Fun;
extern Shoot_Data_t Shoot_Data;

extern Struct_PID_Manage_Object_Fuzzy M3508_FricL_Pid;
extern Struct_PID_Manage_Object_Fuzzy M3508_FricR_Pid;
extern Struct_PID_Manage_Object_Fuzzy M3508_FricU_Pid;
extern positionpid_t M3508_DialV_Pid;        
extern incrementalpid_t M3508_DialI_Pid;

#endif /*__SHOOT_H*/
