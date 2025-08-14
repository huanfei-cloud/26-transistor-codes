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
#include "J4310_Motor.h"
#include "M6020_Motor.h"
#include "DT7.h"
#include "typedef.h"
//#include <AddMath.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//���°�װ��������ô���ʱ��Ҫ���²�����Щֵ��toalAngle�����������˶���

#define Cloud_Yaw_Center 1500
#define Cloud_Yaw_ReverseCenter 7809

//#define Cloud_Pitch_MIN = 100;
//#define Cloud_Pitch_MIN= -500;    //Pitch�����
//#define Cloud_Pitch_MAX = 380;

#define Cloud_Pitch_level 0

/* ��̨����ǶȽṹ�� */
typedef struct
{
    float Yaw_Raw; 			//yaw��ԭʼ����
    float Pitch_Raw;   		//pitch��ԭʼ����
    float Target_Yaw; 		//��̨Ŀ��yaw��
    float Target_Pitch;   	//��̨Ŀ��pitch��
		float AutoAim_Pitch;      //yaw�������
		float AutoAim_Yaw;        //pitch�������
	float Vision_Yaw_Delta;		//�Ӿ�Yaw������(��ֵ)
	float Vision_Pitch_Delta;		//�Ӿ�Pitch������(��ֵ)
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
} Cloud_FUN_t;

void Cloud_Init(void);

extern int16_t Aim_Yaw_RawAngle;
extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;


#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
			&Cloud_Sport_Out,			  \
    }


#endif /* __CLOUD_CONTROL_H */
