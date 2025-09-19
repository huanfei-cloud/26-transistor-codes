/**
 * @file Dial.c
 * @author Why,ZS
 * @brief 处理拨盘电机的控制问题
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Dial.h"
#include "FuzzyPID.h"
#include "PID.h"
#include "FeedForward.h"

/**************用户数据定义****************/
void Dial_Processing(void);
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void PID_Clear_Dial(void);
float PID_Model4_Update(incrementalpid_t *pid,FUZZYPID_Data_t *PID, float _set_point, float _now_point);

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
void Dial_Processing(void)
{	
	if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
	{
		M2006_Array[Dial_Motor].targetSpeed = Dial_Data.Speed_Dial;
	}
	
    M2006_Array[Dial_Motor].outCurrent  = PID_Model4_Update(&M2006_DialI_Pid,
	                                        &fuzzy_pid_bullet_v,
                                          M2006_Array[Dial_Motor].targetSpeed,
                                          M2006_Array[Dial_Motor].realSpeed);
}

/**
  * @brief  更新拨盘电机的角度值
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Update_Angel(bool Fric_ReadyOrNot)
{
}

/**
  * @brief  Dial电机PID清除
  * @param  void
  * @retval void
  * @attention
  */
void PID_Clear_Dial(void)
{

	Position_PIDInit(&M2006_DialV_Pid, 0.3f, 0.001f, 0.4, 0.5, 20000, 8000, 700);//拨盘电机速度环
	Incremental_PIDInit(&M2006_DialI_Pid, 12.5f, 0.5f, 0, 10000, 1000);//拨盘电机电流环
}

