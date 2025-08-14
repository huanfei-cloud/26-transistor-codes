/**
 * @file Dial.c
 * @author Why
 * @brief �����̵���Ŀ�������
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

/**************�û����ݶ���****************/
void Dial_Processing(void);
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void Dial_Back_OneBullet(void);
void Dial_OneBullet(void);

/****************�ӿڶ���******************/
Dial_Fun_t Dial_Fun = Dial_FunGroundInit;
#undef Fric_FunGroundInit
Dial_Data_t Dial_Data = Dial_DataGroundInit;
#undef Dial_DataGroundInit

/**
  * @brief  ���̵���ܴ�����
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Processing()
{
	static uint8_t times = 5;   //��������ٶ�pid������һ��λ��pid
	
	if (Dial_Data.Shoot_Mode == Single_Shoot)
	{
		times++;
		if (times >= 5)
		{
			M3508_Array[Dial_Motor].targetSpeed = Position_PID_Dial(&M3508_DialV_Pid, &FuzzyPID_Dial,
												  M3508_Array[Dial_Motor].targetAngle,
												  M3508_Array[Dial_Motor].totalAngle)/3;//����3��Ϊ�˼����ٶ�
			times = 0;
		}
	}
	M3508_Array[Dial_Motor].outCurrent  = Incremental_PID(&M3508_DialI_Pid, 
															M3508_Array[Dial_Motor].targetSpeed,
															M3508_Array[Dial_Motor].realSpeed);
	//FeedForward_FUN.FeedForward_Dial();//ע�͵��󣬿�ʼ����2024.01.06 lxr
	}

	/**
  * @brief  �˵�
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
  * @brief  ����
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
  * @brief  ���²��̵���ĽǶ�ֵ
  * @param  void
  * @retval void
  * @attention
  */
void Dial_Update_Angel(bool Fric_ReadyOrNot)
{	
	//Ħ����ת�������䶨�ٶ� 
	if(Fric_ReadyOrNot)
	{
		//����������һ��42mm
		if(ControlMes.shooter_42mm_heat_now + 100 < ControlMes.shooter_42mm_heat_limit )
		{
			M3508_Array[Dial_Motor].targetAngle -= (float)Angle_DialOneBullet_42mm;
			Dial_Data.Number_ToBeFired-- ;
			Dial_Data.Bullet_Dialed++ ;
		}
	}
}
