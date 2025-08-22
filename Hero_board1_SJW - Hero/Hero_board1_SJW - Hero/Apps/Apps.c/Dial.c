/**
 * @file Dial.c
 * @author Why,ZS,SJW
 * @brief 拨弹盘处理总函数
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Dial.h"

/**************相关函数定义****************/
void Dial_Processing_2(void);
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void Dial_Back_OneBullet(void);
void Dial_OneBullet(void);
float PID_Model4_Update(incrementalpid_t *pid, FUZZYPID_Data_t *PID, float _set_point, float _now_point);

/****************函数结构体声明******************/
Dial_Fun_t Dial_Fun = Dial_FunGroundInit;
#undef Dial_FunGroundInit
Dial_Data_t Dial_Data = Dial_DataGroundInit;
#undef Dial_DataGroundInit

/**
 * @brief  拨弹处理函数2
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_2(void)
{

	static uint32_t last_dial_time = 0;
	uint32_t current_time = HAL_GetTick();

	/****************拨弹*****************/
	if (Dial_Data.Shoot_Mode == Single_Shoot && Dial_Data.Dial_Switch == Dial_On)
	{
		// 检查是否到达2秒间隔
		if (current_time - last_dial_time >= 2000) // 2000ms = 2秒
		{
			Dial_OneBullet();
			last_dial_time = current_time;
		}
		M3508_Array[Dial_Wheel].targetSpeed = Position_PID_Dial(&M3508_DialV_Pid, &FuzzyPID_Dial, M3508_Array[Dial_Wheel].targetAngle, M3508_Array[Dial_Wheel].totalAngle);
		M3508_Array[Dial_Wheel].outCurrent = Incremental_PID(&M3508_DialI_Pid, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
	}

	/*****************退弹***************/
	else if (Dial_Data.Shoot_Mode == Single_Shoot && Dial_Data.Dial_Switch == Dial_Back)
	{

		Dial_Back_OneBullet();
		M3508_Array[Dial_Wheel].targetSpeed = Position_PID_Dial(&M3508_DialV_Pid, &FuzzyPID_Dial, M3508_Array[Dial_Wheel].targetAngle, M3508_Array[Dial_Wheel].totalAngle);
		M3508_Array[Dial_Wheel].outCurrent = Incremental_PID(&M3508_DialI_Pid, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
	}

	else
	{
		M3508_Array[Dial_Wheel].targetSpeed = 0;
		M3508_Array[Dial_Wheel].outCurrent = Incremental_PID(&M3508_DialI_Pid, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
	}
}

/**
 * @brief  退弹
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Back_OneBullet()
{
	M3508_Array[Dial_Wheel].targetAngle += (float)Angle_DialOneBullet_42mm;
}

/**
 * @brief  单发
 * @param  void
 * @retval void
 * @attention
 */
void Dial_OneBullet()
{
	M3508_Array[Dial_Wheel].targetAngle -= (float)Angle_DialOneBullet_42mm;
}

/**
 * @brief  更新拨盘电机的角度值
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Update_Angel(bool Fric_ReadyOrNot)
{
	// 摩擦轮转速满足射定速度
	if (Fric_ReadyOrNot)
	{
		Dial_Fun.Dial_Processing_2();
		Dial_Data.Bullet_Dialed++;
	}
	else
	{
		M3508_Array[Dial_Wheel].targetSpeed = 0;
		M3508_Array[Dial_Wheel].outCurrent = Incremental_PID(&M3508_DialI_Pid, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
	}
}

