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

incrementalpid_t M3508_FricF1_Pid; // 前1pid
incrementalpid_t M3508_FricF2_Pid; // 前2pid
incrementalpid_t M3508_FricF3_Pid; // 前3pid
incrementalpid_t M3508_FricB1_Pid; // 后1pid
incrementalpid_t M3508_FricB2_Pid; // 后2pid
incrementalpid_t M3508_FricB3_Pid; // 后3pid

positionpid_t M3508_DialV_Pid;	  // 外环pid
incrementalpid_t M3508_DialI_Pid; // 内环pid

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
		M3508_Array[Dial_Wheel].targetAngle = M3508_Array[Dial_Wheel].totalAngle;
	if (Shoot_Data.Shoot_Switch == Shoot_On)
	{
		Dial_Data.Dial_Switch = Dial_On;
	}
	else if (Shoot_Data.Shoot_Switch == Shoot_Off)
	{
		Dial_Data.Dial_Switch = Dial_Off;
	}
	else if(Shoot_Data.Shoot_Switch == Shoot_Single)
	{
		Dial_Data.Dial_Switch = Dial_Single;
	}

	/* 不用拨弹 */
	if (ControlMes.shoot_state == RC_SW_MID)
	{
		if (ControlMes.redial == 1)
		{
			M3508_Array[Dial_Wheel].targetSpeed = 2000;
		}
		else
		{
			M3508_Array[Dial_Wheel].targetSpeed = 0;
		}
	}
}
