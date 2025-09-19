/**
 * @file Cloud_control.c
 * @author Cyx，ZS, SJW
 * @brief
 * @version 2.5
 * @date 2023-08-15
 *
 * @copyright
 *
 */

#include "Cloud_Control.h"

// 重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

/****************Pithch限位*****************/
const float Delta_Pitch_Min = -10;
const float Delta_Pitch_Max = 65;
const float Cloud_Pitch_Min = -10;
const float Cloud_Pitch_Max = 65;
const float Pitch_Angle_Init = 0;
const float Cloud_Pitch_Center = 0;
const float Cloud_Pitch_Derta = Cloud_Pitch_Center - Cloud_Pitch_Min;
/****************Pitch限位  End*****************/

/*****************数据声明****************/
Cloud_t Cloud;
float Cloud_Target_Aim_Flag;
float Cloud_Init_Angle;
extern Saber_Angle_t Saber_Angle;
float Pitch_Torque = 3.f; // 云台所需扭矩
float Pitch_v = 2;
float Pitch_Kp = 60;
float Pitch_Kd = 2;
float Pitch_RC_Sen = 0.001;
int16_t Cloud_Aim_Pitch_Flag;
int Aim_Flag = 0;
/*****************数据声明end****************/

/******************卡尔曼滤波结构体创建*********/
One_Kalman_t Cloud_PitchMotorAngle_Error_Kalman; // Pitch轴电机角度误差的Kalman滤波器
One_Kalman_t Cloud_PitchCurrent_Kalman;			 // Pitch轴电流的Kalman滤波器
/******************卡尔曼滤波结构体创建 end*********/

/*****************函数声明****************/
void Cloud_Init(void);
void Cloud_Pitch_Angle_Set(void);
void Cloud_Sport_Out(void);
void Remote_Change(void);
/*****************函数声明end****************/

/***************输出接口定义***************/
Cloud_FUN_t Cloud_FUN = Cloud_FUNGroundInit;
#undef Cloud_FUNGroundInit

/**
 * @brief  云台初始化，配置参数并归位云台
 * @param  None
 * @retval None
 */
void Cloud_Init(void)
{
	// 保存启动时刻的机械角度
	Cloud.Target_Pitch = Cloud_Pitch_Min;
	Cloud.Pitch_Raw = Cloud_Pitch_Min;
	Cloud.AutoAim_Pitch = 0;

	One_Kalman_Create(&Cloud_PitchMotorAngle_Error_Kalman, 1.5, 40);
	One_Kalman_Create(&Cloud_PITCHODKalman, 1, 10);
	One_Kalman_Create(&Cloud_PitchCurrent_Kalman, 6, 10);
	ControlMes.change_Flag = 0;
	ControlMes.shoot_Speed = 2;
	ControlMes.fric_Flag = 0;
	ControlMes.redial = 0;
}

/**
 * @brief  低通滤波
 * @param  float float float 滤波对象，上一次滤波后的值，滤波系数
 * @retval void
 * @attention
 */
float low_pass_filter(float current_value, float prev_value, float alpha)
{
	return alpha * current_value + (1 - alpha) * prev_value;
}

/**
 * @brief  J4340_Pitch电机角度调整，修正电机电流数据
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Pitch_Angle_Set(void)
{
	/****************************云台pitchJ4340电机******************************/
	/******************************遥控器数值传递******************************/
	static float Delta_Pitch = 0;
	if (ControlMes.AutoAimFlag == 1)
	{
		Delta_Pitch += (float)ControlMes.pitch_velocity * Pitch_RC_Sen;

		/**********Delta_Pitch限位**********/
		if (Delta_Pitch > Delta_Pitch_Max)
		{
			Delta_Pitch = Delta_Pitch_Max;
		}
		else if (Delta_Pitch < Delta_Pitch_Min)
		{
			Delta_Pitch = Delta_Pitch_Min;
		}
		/**********Delta_Pitch限位end**********/

		Cloud.Target_Pitch = Pitch_Angle_Init + Cloud.AutoAim_Pitch + Delta_Pitch;
		Aim_Flag = 1;
	}

	else
	{
		Delta_Pitch += (float)ControlMes.pitch_velocity * Pitch_RC_Sen;

		/**********Delta_Pitch限位**********/
		if (Delta_Pitch > Delta_Pitch_Max)
		{
			Delta_Pitch = Delta_Pitch_Max;
		}
		else if (Delta_Pitch < Delta_Pitch_Min)
		{
			Delta_Pitch = Delta_Pitch_Min;
		}
		/**********Delta_Pitch限位end**********/

		if (Aim_Flag == 0)
		{
			Cloud.Target_Pitch = Delta_Pitch + Pitch_Angle_Init;
		}
		else if (Aim_Flag == 1)
		{
			Cloud.Target_Pitch = Delta_Pitch + Pitch_Angle_Init;
		}
	}

	/**********Pitch限位**********/
	if (Cloud.Target_Pitch > Cloud_Pitch_Max)
	{
		Cloud.Target_Pitch = Cloud_Pitch_Max;
	}
	else if (Cloud.Target_Pitch < Cloud_Pitch_Min)
	{
		Cloud.Target_Pitch = Cloud_Pitch_Min;
	}

	/**************************Pitch轴电机控制，达秒J4340电机MIT模式，参数赋值*****************/
	J4340s_Pitch.outKp = Pitch_Kp;
	J4340s_Pitch.outKd = Pitch_Kd;
	J4340s_Pitch.outSpeed = Pitch_v;
	J4340s_Pitch.outTorque = Pitch_Torque;
	One_Kalman_Filter(&Cloud_PitchCurrent_Kalman, Cloud.Target_Pitch);
	J4340s_Pitch.outPosition = Cloud.Target_Pitch;
}

/**
 * @brief  J4340电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Sport_Out(void)
{
	Cloud_Pitch_Angle_Set();
}

/**
 * @brief  变速小陀螺
 * @param  void
 * @retval void
 * @attention
 */

void Remote_Change(void)
{
	static int change_State = 0;
	static int change_Remote = 0;
	if (ControlMes.change_Flag == 1)
	{
		if (change_State == 0)
		{
			change_Remote += 3;
			ControlMes.z_rotation_velocity += 3;
			if (change_Remote >= 300)
			{
				change_State = 1;
			}
		}
		else if (change_State == 1)
		{
			change_Remote -= 3;
			ControlMes.z_rotation_velocity -= 3;
			if (change_Remote <= -300)
			{
				change_State = 0;
			}
		}
	}
	else
	{
		ControlMes.z_rotation_velocity -= change_Remote;
		change_Remote = 0;
	}
}
