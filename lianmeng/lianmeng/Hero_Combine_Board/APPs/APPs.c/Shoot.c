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
#include "BSP_Fdcan.h"
#include "Cloud_Control.h"
#include "Extern_Handles.h"

/**************用户数据定义****************/
void Shoot_Processing(void);

/****************接口定义******************/
Shoot_Fun_t Shoot_Fun = Shoot_FunGroundInit;
#undef Shoot_FunGroundInit
Shoot_Data_t Shoot_Data = Shoot_DataGroundInit;
#undef Shoot_DataGroundInit

Struct_PID_Manage_Object_Fuzzy M3508_FricL_Pid;     //左摩擦轮电机pid
Struct_PID_Manage_Object_Fuzzy M3508_FricR_Pid;     //右摩擦轮电机pid
Struct_PID_Manage_Object_Fuzzy M3508_FricU_Pid;     //上摩擦轮电机pid
positionpid_t M3508_DialV_Pid;        //拨盘电机速度pid
incrementalpid_t M3508_DialI_Pid;     //拨盘电机电流pid

/**
  * @brief  射击总处理函数
  * @param  void
  * @retval void
  * @attention
  */
void Shoot_Processing()
{
	
	/* 处理摩擦轮 */
	Fric_Fun.Fric_Processing();  
	Fric_Fun.Fric_Judge_ReadyOrNot(); 	
//	/* 处理拨盘电机 */
	if(Dial_Data.Number_ToBeFired) // 如果需要发弹
	{
		Dial_Fun.Dial_Update_Angel(Fric_Data.Fric_Ready);
	}
	Dial_Fun.Dial_Processing();            
	

}

