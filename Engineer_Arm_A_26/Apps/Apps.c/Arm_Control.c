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

/****位置环PID参数设置****/

//J3电机（小臂电机）
#define J3_kp 1.0f
#define J3_ki 0.0001f
#define J3_kd 0.1f
#define J3_kf 0.0f
//J4电机（大Roll电机）
#define J4_kp 1.0f
#define J4_ki 0.0001f
#define J4_kd 0.1f
#define J4_kf 0.0f
//J5电机（Pitch电机）
#define J5_kp 1.0f
#define J5_ki 0.0001f
#define J5_kd 0.1f
#define J5_kf 0.0f
//J6电机（小Roll电机）
#define J6_kp 1.0f
#define J6_ki 0.0001f
#define J6_kd 0.1f
#define J6_kf 0.0f

/****MIT模式数据初始化设置****/

//J4340电机
float J4340_T = 1.0f; 
float J4340_P = 0.0f; 
float J4340_V = 0.0f;
float J4340_Kp = 0.0f; //目标位置给零且KP给零直接转为速度模式
float J4340_Kd = 1.0f;
//J4310电机
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
	
	  PID_Init(&(J4340s_J3.l_pid_object),J3_kp,J3_ki,J3_kd,J3_kf,5.0f,J4340_MaxV,0.1f);
	  PID_Init(&(J4310s_J4.l_pid_object),J4_kp,J4_ki,J4_kd,J4_kf,5.0f,J4310_MaxV,0.1f);
	  PID_Init(&(J4310s_J5.l_pid_object),J5_kp,J5_ki,J5_kd,J5_kf,5.0f,J4310_MaxV,0.1f);
	  PID_Init(&(J4310s_J6.l_pid_object),J6_kp,J6_ki,J6_kd,J6_kf,5.0f,J4310_MaxV,0.1f);
	
	  DM_Enable(0x04,&hcan2);
	  DM_Enable(0x05,&hcan2);
	  DM_Enable(0x06,&hcan2);
	  DM_Enable(0x07,&hcan2);
}

/**
 * @brief 机械臂电机目标位置计算
 * @param None
 * @param None
 */
void Arm_Motor_Set_Location(void)
{
	//J3区域转动
	J4340s_J3.targetAngle = Arm_Data.J3_targetlocation;
	J4340s_J3.outPosition = J4340s_J3.targetAngle * J4340_MaxP / 8192.0f;
	//J4整周转动
	J4310s_J4.targetAngle = Arm_Data.J4_targetlocation;
	J4310s_J4.outPosition = J4310s_J4.targetAngle * J4310_MaxP / 8192.0f;
	//J5区域转动
	J4310s_J5.targetAngle = Arm_Data.J5_targetlocation;
	J4310s_J4.outPosition = J4310s_J4.targetAngle * J4310_MaxP / 8192.0f;
	//J6整周转动
	J4310s_J6.targetAngle = Arm_Data.J6_targetlocation;
	J4310s_J4.outPosition = J4310s_J4.targetAngle * J4310_MaxP / 8192.0f;
}

/**
 * @brief 机械臂电机目标速度
 * @param None
 * @param None
 */
void Arm_Motor_Set_Velocity(void)
{
  DMmotor_location_change(&J4340s_J3,pid_control2,J4340s_J3.targetAngle,J4340s_J3.realAngle);
	DMmotor_location_change(&J4310s_J4,pid_control2,J4310s_J4.targetAngle,J4310s_J4.realAngle);
	DMmotor_location_change(&J4340s_J3,pid_control2,J4340s_J3.targetAngle,J4340s_J3.realAngle);
	DMmotor_location_change(&J4310s_J4,pid_control2,J4310s_J4.targetAngle,J4310s_J4.realAngle);
	
	
}