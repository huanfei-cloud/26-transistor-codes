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
#include "shoot.h"
#include "DT7.h"
#include "typedef.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//���°�װ��������ô���ʱ��Ҫ���²�����Щֵ��toalAngle�����������˶���
/****************Pitch��λ*****************/
#define Cloud_Pitch_level 3614
#define Cloud_Pitch_Init 4280
#define Cloud_Pitch_Min 2900
#define Cloud_Pitch_Max 4280
/****************Pithch��λEnd*****************/


/* ��̨����ǶȽṹ�� */
typedef struct
{
    float Yaw_Raw;         //yaw��ԭʼ����
    float Pitch_Raw;       //pitch��ԭʼ����
    float Target_Yaw;      //��̨Ŀ��yaw��
    float Target_Pitch;    //��̨Ŀ��pitch��
	float AutoAim_Pitch;   //����õ���pithc��Ƕ�
} Cloud_t;


typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_Sport_Out)(void);
    void (*PID_Clear_Pitch)(void);
		void (*Cloud_Pitch_Angle_Set)(void);
		void (*Remote_Change)(void);

} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;

#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
		&Cloud_Sport_Out,			 	  						\
			&PID_Clear_Pitch,										\
			&Cloud_Pitch_Angle_Set,  	\
			&Remote_Change,				\
    }


#endif /* __CLOUD_CONTROL_H */
