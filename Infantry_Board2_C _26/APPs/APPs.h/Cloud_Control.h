/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief 
 * @version 0.1
 * @date 2023-08-15
 * 
 * @copyright 
 * 
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "PID.h"
#include "kalman_filter.h"
#include "M6020_Motor.h"
#include "typedef.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FuzzyPID.h"
#include "FeedForward.h"
#include "Omni_Chassis.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"
#include "BSP_BoardCommunication.h"
#include "Saber_C3.h"
#include "Gimbal_Chassis_Pitch_Angle.h"

//���°�װ��������ô���ʱ��Ҫ���²�����Щֵ��toalAngle�����������˶���

#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809



/* ��̨����ǶȽṹ�� */
typedef struct
{
    float Yaw_Raw; 			//yaw��ԭʼ����
    float Pitch_Raw;   		//pitch��ԭʼ����
    float Target_Yaw; 		//��̨Ŀ��yaw��
    float Target_Pitch;   	//��̨Ŀ��pitch��
	float Vision_Yaw_Delta;		//�Ӿ�Yaw������(��ֵ)
	float Vision_Pitch_Delta;		//�Ӿ�Pitch������(��ֵ)
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
		void (*Cloud_Yaw_Angle_Set)(void);
		void (*PID_Clear_Yaw)(void);
} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;
void Gimbal_Pitch_Translate(void);


#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
		&Cloud_Sport_Out,			  	  \
			&Cloud_Yaw_Angle_Set,			\
			&PID_Clear_Yaw,					\
    }


#endif /* __CLOUD_CONTROL_H */
