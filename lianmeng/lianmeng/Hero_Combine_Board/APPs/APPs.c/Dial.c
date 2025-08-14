/**
 * @file Dial.c
 * @author Why
 * @brief 处理拨盘电机的控制问题
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Dial.h"
#include "FuzzyPID.h"
#include "FeedForward.h"
#include "BSP_BoardCommunication.h"

/**************用户数据定义****************/
void Dial_Processing(void);
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void Dial_Back_OneBullet(void);
void Dial_OneBullet(void);

/****************接口定义******************/
Dial_Fun_t Dial_Fun = Dial_FunGroundInit;
#undef Fric_FunGroundInit
Dial_Data_t Dial_Data = Dial_DataGroundInit;
#undef Dial_DataGroundInit

/**
  * @brief  拨盘电机总处理函数
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Processing()
{
	static uint8_t times = 5;   //计算五次速度pid后再算一次位置pid
	
	if (Dial_Data.Shoot_Mode == Single_Shoot)
	{
		times++;
		if (times >= 5)
		{
			M3508_Array[Dial_Motor].targetSpeed = Position_PID_Dial(&M3508_DialV_Pid, &FuzzyPID_Dial,
												  M3508_Array[Dial_Motor].targetAngle,
												  M3508_Array[Dial_Motor].totalAngle)/3;//除以3是为了减慢速度
			times = 0;
		}
	}
	M3508_Array[Dial_Motor].outCurrent  = Incremental_PID(&M3508_DialI_Pid, 
															M3508_Array[Dial_Motor].targetSpeed,
															M3508_Array[Dial_Motor].realSpeed);
	//FeedForward_FUN.FeedForward_Dial();//注释掉后，开始拨动2024.01.06 lxr
	}

	/**
  * @brief  退弹
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Back_OneBullet()
{
	M3508_Array[Dial_Motor].targetAngle += (float)Angle_DialOneBullet_42mm;
	Dial_Data.Number_ToBeFired = 0;
}
/**
  * @brief  单发
  * @param  void
  * @retval void
  * @attention
  */
void Dial_OneBullet()
{
	M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
	Dial_Data.Number_ToBeFired = 0;
}

/**
  * @brief  更新拨盘电机的角度值
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Update_Angel(bool Fric_ReadyOrNot)
{	
	//摩擦轮转速满足射定速度 
	if(Fric_ReadyOrNot)
	{
		//热量允许发射一发42mm
		if(ControlMes.shooter_42mm_heat_now + 100 < ControlMes.shooter_42mm_heat_limit )
		{
			M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
			Dial_Data.Number_ToBeFired-- ;
			Dial_Data.Bullet_Dialed++ ;
		}
	}
}
