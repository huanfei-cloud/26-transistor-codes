/**
 * @file Shoot.c
 * @author Why
 * @brief �ۺ�Ħ���ֺͲ��̵���Ĵ������������������١�����������
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Shoot.h"
#include "BSP_Fdcan.h"
#include "Cloud_Control.h"
#include "Extern_Handles.h"

/**************�û����ݶ���****************/
void Shoot_Processing(void);

/****************�ӿڶ���******************/
Shoot_Fun_t Shoot_Fun = Shoot_FunGroundInit;
#undef Shoot_FunGroundInit
Shoot_Data_t Shoot_Data = Shoot_DataGroundInit;
#undef Shoot_DataGroundInit

Struct_PID_Manage_Object_Fuzzy M3508_FricL_Pid;     //��Ħ���ֵ��pid
Struct_PID_Manage_Object_Fuzzy M3508_FricR_Pid;     //��Ħ���ֵ��pid
Struct_PID_Manage_Object_Fuzzy M3508_FricU_Pid;     //��Ħ���ֵ��pid
positionpid_t M3508_DialV_Pid;        //���̵���ٶ�pid
incrementalpid_t M3508_DialI_Pid;     //���̵������pid

/**
  * @brief  ����ܴ�����
  * @param  void
  * @retval void
  * @attention
  */
void Shoot_Processing()
{
	
	/* ����Ħ���� */
	Fric_Fun.Fric_Processing();  
	Fric_Fun.Fric_Judge_ReadyOrNot(); 	
//	/* �����̵�� */
	if(Dial_Data.Number_ToBeFired) // �����Ҫ����
	{
		Dial_Fun.Dial_Update_Angel(Fric_Data.Fric_Ready);
	}
	Dial_Fun.Dial_Processing();            
	

}

