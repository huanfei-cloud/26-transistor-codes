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
	Fric_Fun.Fric_Processing();						  // 摩擦轮处理函数
	Fric_Fun.Fric_Judge_ReadyOrNot();				  // 检测摩擦轮是否就绪
	Dial_Fun.Dial_Update_Angel(Fric_Data.Fric_Ready); // 拨弹盘角度更新并处理
}
