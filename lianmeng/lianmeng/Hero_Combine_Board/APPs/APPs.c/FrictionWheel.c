/**
 * @file FrictionWheel.c
 * @author Why
 * @brief ����Ħ���ֵĿ�������
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "FrictionWheel.h"
#include "FeedForward.h"

/**************�û����ݶ���****************/
void Fric_Processing(void);
void Fric_Judge_ReadyOrNot(void);
void Fric_Set_targetSpeed(void);

/****************�ӿڶ���******************/
Fric_Fun_t Fric_Fun = Fric_FunGroundInit;
#undef Fric_FunGroundInit
Fric_Data_t Fric_Data = Fric_DataGroundInit;
#undef Fric_DataGroundInit

fp32 Fric_SpeedLevel1 = 5200;


/**
  * @brief  Ħ���ֿ����ܴ�����
  * @param  void
  * @retval void
  * @attention
  */
void Fric_Processing()
{
	/* �趨Ŀ��ֵ */
	Fric_Set_targetSpeed();
	FeedForward_FUN.FeedForward_Fric();
	
	M3508_Array[FricL_Wheel].targetSpeed = -1*Fric_Data.Required_Speed;
	M3508_Array[FricR_Wheel].targetSpeed = Fric_Data.Required_Speed;
	M3508_Array[FricU_Wheel].targetSpeed = Fric_Data.Required_Speed;
	
	
	M3508_Array[FricL_Wheel].outCurrent = PID_Model4_Update(&M3508_FricL_Pid, 
															&fuzzy_pid_fric_l,
														  M3508_Array[FricL_Wheel].targetSpeed, 
														  M3508_Array[FricL_Wheel].realSpeed);
	M3508_Array[FricR_Wheel].outCurrent = PID_Model4_Update(&M3508_FricR_Pid, 
															&fuzzy_pid_fric_r,
														  M3508_Array[FricR_Wheel].targetSpeed,
														  M3508_Array[FricR_Wheel].realSpeed);
	M3508_Array[FricU_Wheel].outCurrent = PID_Model4_Update(&M3508_FricU_Pid, 
															&fuzzy_pid_fric_u,
														  M3508_Array[FricU_Wheel].targetSpeed,
														  M3508_Array[FricU_Wheel].realSpeed);

}

/**
  * @brief  ��Ħ���ֵ��ٶȴﵽtarget����ʱ��ΪĦ�����Ѿ�����
  * @param  void
  * @retval void
  * @attention
  */
void Fric_Judge_ReadyOrNot()
{
	static q15_t abs_differ[2] = {0};
	static q15_t abs_factor[2];           //��Ŀ���ٶȵ�0.05Ϊ��
	static q15_t temp[3];
	
	temp[0] = Fric_Data.Required_Speed - M3508_Array[FricL_Wheel].realSpeed;
	temp[1] = Fric_Data.Required_Speed + M3508_Array[FricR_Wheel].realSpeed;
	temp[2] = Fric_Data.Required_Speed + M3508_Array[FricU_Wheel].realSpeed;
	arm_abs_q15(temp, abs_differ, 3);
	temp[0] = Fric_Data.Required_Speed * 0.05f;
	temp[1] = Fric_Data.Required_Speed * 0.05f;
	temp[2] = Fric_Data.Required_Speed * 0.05f;
	arm_abs_q15(temp, abs_factor, 3);
	
	if(abs_differ[0] < abs_factor[0] && abs_differ[1] < abs_factor[1])
		Fric_Data.Fric_Ready = Fric_Ready;
	else Fric_Data.Fric_Ready = Fric_NotReady;
}

/**
  * @brief  �趨Ħ���ֵ�Ŀ���ٶ�
  * @param  void
  * @retval void
  * @attention
  */
void Fric_Set_targetSpeed()
{
	if(Fric_Data.Fric_Switch == Fric_Off)
	{
		Fric_Data.Required_Speed = 0;
		return;
	}
	else if(Fric_Data.Fric_Switch == Fric_On)
	{
		Fric_Data.Required_Speed = Fric_SpeedLevel1;
	}
}
