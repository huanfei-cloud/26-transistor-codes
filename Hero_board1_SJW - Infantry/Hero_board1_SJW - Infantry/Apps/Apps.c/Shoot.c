/**
 * @file Shoot.c
 * @author Why
 * @brief 综合摩擦轮和拨盘电机的处理，并处理热量、射速、卡弹等问题
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Shoot.h"

/**************用户数据定义****************/
void Shoot_Processing(void);
void Shoot_Update_Status(void);

/****************接口定义******************/
Shoot_Fun_t Shoot_Fun = Shoot_FunGroundInit;
#undef Shoot_FunGroundInit
Shoot_Data_t Shoot_Data = Shoot_DataGroundInit;
#undef Shoot_DataGroundInit

incrementalpid_t M3508_FricL_Pid; // 左摩擦轮电机pid
incrementalpid_t M3508_FricR_Pid; // 右摩擦轮电机pid
positionpid_t M2006_DialV_Pid;	  // 拨盘电机速度pid
incrementalpid_t M2006_DialI_Pid; // 拨盘电机电流pid

/**
 * @brief  射击总处理函数
 * @param  void
 * @retval void
 * @attention
 */
void Shoot_Processing()
{
	Fric_Fun.Fric_Processing();

	Shoot_Update_Status();
	/* 处理拨盘电机 */
	Dial_Fun.Dial_Processing_2();
}

/**
 * @brief  射击相关各个状态位的更新
 * @param  void
 * @retval void
 * @attention
 */
void Shoot_Update_Status()
{
	if (Dial_Data.Shoot_Mode == Continuous_Shoot) // 不用单发时把目标设置成实际值
		M2006_Array[Dial_Motor].targetAngle = M2006_Array[Dial_Motor].totalAngle;
	if (Shoot_Data.Shoot_Switch == Shoot_On)
	{
		Dial_Data.Dial_Switch = Dial_On;
	}
	else if (Shoot_Data.Shoot_Switch == Shoot_Off)
	{
		Dial_Data.Dial_Switch = Dial_Off;
	}

	/* 不用拨弹 */
	if (ControlMes.shoot_state == RC_SW_MID)
	{
		if (ControlMes.redial == 1)
		{
			M2006_Array[Dial_Motor].targetSpeed = 2000;
		}
		else
		{
			M2006_Array[Dial_Motor].targetSpeed = 0;
		}
	}
}
