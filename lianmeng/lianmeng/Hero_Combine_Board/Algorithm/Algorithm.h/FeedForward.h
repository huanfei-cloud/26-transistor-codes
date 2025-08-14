/**
 * @file FeedForward.h
 * @author Why
 * @brief ǰ������
 * @version 0.1
 * @date 2023-08-28
 * 
 * @copyright 
 * 
 */

#ifndef __FEEdFORWARD_H
#define __FEEdFORWARD_H

#include "main.h"

#define Pitch_Fitting 6       //Pitch�����ĺ�����Ϸ�ʽ��2��ʾ���Σ�3��ʾ���ǣ�6��ʾ6�ζ���ʽ
#define Pitch_Margin 15			//�����뾶
#define MAX_GRAVITY 10          //����������ƽ�������ĸ���
#define Num_InTube 4            //�������ֱ���е��ӵ�����
#define Gravity_Para 57.1f      //�������������ǰ������
#define Accelerate_Para 0.0641f //����������Ǽ��ٱȺ�ļ��ٶȲ������� 

#define FeedForward_FunGroundInit    \
    {                                \
		&FeedForward_Fric,           \
		&FeedForward_Dial,        \
		&FeedForward_Pitch,      \
		&FeedForward_Pitch_Accelerate	\
    }

typedef struct
{
    void (*FeedForward_Fric)(void);
	void (*FeedForward_Dial)(void);
	void (*FeedForward_Pitch)(void);
	void (*FeedForward_Pitch_Accelerate)(void);
} FeedForward_FUN_t;

extern FeedForward_FUN_t FeedForward_FUN;
#endif
