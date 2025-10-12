/**
 * @file Arm_Control.c
 * @author xhf
 * @brief 机械臂控制
 * @version 1.0
 * @date 2025-08-21
 * @copyright Transistor BUAA
 */

#include "Arm_Control.h"

/********变量定义********/
Arm_Data_t  Arm_Data;

//MIT模式数据初始化设置
//J4340电机
float J4340_T = 1.0f; 
float J4340_P = 0.0f; 
float J4340_V = 0.0f;
float J4340_Kp = 0.0f; //目标位置给零且KP给零直接转为速度模式
float J4340_Kd = 1.0f;
//J4340电机
float J4310_T = 1.0f; 
float J4310_P = 0.0f; 
float J4310_V = 0.0f;
float J4310_Kp = 0.0f; //目标位置给零且KP给零直接转为速度模式
float J4310_Kd = 1.0f;


/**
 * @brief 机械臂电机初始化
 * @param None
 * @param None
 */
void Arm_Motor_Init(void)
{
    DM_MIT_Init(&J4340s_J3,J4340_T,J4340_P,J4340_V,J4340_Kp,J4340_Kd);
	  DM_MIT_Init(&J4310s_J4,J4310_T,J4310_P,J4310_V,J4310_Kp,J4310_Kd);
	  DM_MIT_Init(&J4310s_J5,J4310_T,J4310_P,J4310_V,J4310_Kp,J4310_Kd);
	  DM_MIT_Init(&J4310s_J6,J4310_T,J4310_P,J4310_V,J4310_Kp,J4310_Kd);
	
	  DM_Enable(0x04,&hcan1);
	  DM_Enable(0x05,&hcan1);
	  DM_Enable(0x06,&hcan2);
	  DM_Enable(0x07,&hcan2);
}


