/**
 * @file FeedForward.h
 * @author Why
 * @brief 前馈控制
 * @version 0.1
 * @date 2023-08-28
 * 
 * @copyright 
 * 
 */

#ifndef __FEEdFORWARD_H
#define __FEEdFORWARD_H

#include "main.h"

#define Pitch_Fitting 6       //Pitch轴电机的函数拟合方式，2表示二次，3表示三角，6表示6次多项式
#define Pitch_Margin 15			//死区半径
#define MAX_GRAVITY 10          //拨弹电机最多平衡重力的个数
#define Num_InTube 4            //拨弹电机直管中的子弹个数
#define Gravity_Para 57.1f      //拨弹电机的重力前馈因子
#define Accelerate_Para 0.0641f //拨弹电机考虑减速比后的加速度补偿因子 

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
