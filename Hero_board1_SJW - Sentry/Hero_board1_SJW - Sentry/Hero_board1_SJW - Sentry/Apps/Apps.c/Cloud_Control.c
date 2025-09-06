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

Cloud_t Cloud;
extern M6020s_t *M6020_Array[1];

// 重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

/****************Pithch限位*****************/
const float Delta_Pitch_Min = -11;
const float Delta_Pitch_Max = 11;
const float Cloud_Pitch_Min = -11;
const float Cloud_Pitch_Max = 11;
const float Pitch_Angle_Init = 0;
const float Cloud_Pitch_Center = 0;
const float Cloud_Pitch_Derta = Cloud_Pitch_Center - Cloud_Pitch_Min;
/****************Pitch限位End*****************/

float Cloud_Target_Aim_Flag;
float Cloud_Init_Angle;
float Pitch_Torque = 3.f; // 云台所需扭矩
float Pitch_v = 20;
float Pitch_Kp = 15;
float Pitch_Kd = 1.5;
float Pitch_RC_Sen = 0.00035;
int16_t Cloud_Aim_Pitch_Flag;
int16_t Cloud_Manual_Pitch_Flag;
int Aim_Flag = 0;
float Delta_Yaw = 0;
float Angle_Yaw_Cloud = 0;
uint8_t kk = 8;

/******************卡尔曼滤波结构体创建*********/
One_Kalman_t Cloud_PitchCurrent_Kalman; // Pitch轴电流的Kalman滤波器
One_Kalman_t Cloud_YawCurrent_Kalman;
/******************卡尔曼滤波结构体创建 end*********/

positionpid_t M6020s_YawIPID;
positionpid_t M6020s_YawOPID;
positionpid_t AutoAim_M6020s_YawIPID;
positionpid_t AutoAim_M6020s_YawOPID;

void Cloud_Init(void);
void Cloud_Pitch_Angle_Set(void);
void Cloud_Sport_Out(void);
void Cloud_Sport_Out_Little_Yaw(void);
void Remote_Change(void);
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
	/**********云台相关代码**********/
	Cloud.Target_Pitch = J4310s_Pitch.outPosition;
	Cloud.Pitch_Raw = J4310s_Pitch.outPosition;
	Cloud_Init_Angle = Pitch_Angle_Init;
	Cloud.AutoAim_Pitch = 0;
	Cloud.Target_Yaw = M6020s_Yaw.realAngle;

	One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 6);
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
 * @brief  J4310_Pitch电机角度调整，修正电机电流数据
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Pitch_Angle_Set(void)
{
	/****************************云台pitchJ4310电机******************************/
	/******************************遥控器数值传递******************************/
	static float Delta_Pitch = 0;
	if (ControlMes.AutoAimFlag == 1)
	{
		Delta_Pitch += (float)ControlMes.pitch_velocity * (-Pitch_RC_Sen);

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
		Delta_Pitch += (float)ControlMes.pitch_velocity * (-Pitch_RC_Sen);

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

	/**************************Pitch轴电机控制，达秒J4310电机MIT模式，参数赋值*****************/
	J4310s_Pitch.outKp = Pitch_Kp;
	J4310s_Pitch.outKd = Pitch_Kd;
	J4310s_Pitch.outSpeed = Pitch_v;
	J4310s_Pitch.outTorque = Pitch_Torque;
	One_Kalman_Filter(&Cloud_PitchCurrent_Kalman, Cloud.Target_Pitch);
	J4310s_Pitch.outPosition = Cloud.Target_Pitch;
}

/**
 * @brief  M6020_Yaw
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Yaw_Angle_Set(void)
{
	/*******************************************************/
	if (M6020s_Yaw.InfoUpdateFrame <= 30)
	{
		Cloud.Target_Yaw = M6020s_Yaw.realAngle;
	}

	Cloud.Target_Yaw += -1 * ControlMes.yaw_velocity * 0.01f;

	// yaw轴目标角度限幅在[-4096,4096]内
	if (Cloud.Target_Yaw > 4096) // 大于4096时，从-4096开始自增
	{
		Cloud.Target_Yaw -= 8192;
	}
	else if (Cloud.Target_Yaw < -4096) // 小于-4096时，从4096开始自减
	{
		Cloud.Target_Yaw += 8192;
	}

	/*******************************************/
	Angle_Yaw_Cloud = M6020s_Yaw.realAngle;

	if (Angle_Yaw_Cloud > 4096) // 大于4096时，从-4096开始自增
	{
		Angle_Yaw_Cloud -= 8192;
	}
	else if (Angle_Yaw_Cloud < -4096) // 小于-4096时，从4096开始自减
	{
		Angle_Yaw_Cloud += 8192;
	}
	ControlMes.yaw_realAngle = Angle_Yaw_Cloud;

	/********************************************* */
	Delta_Yaw = Angle_Yaw_Cloud - Cloud.Target_Yaw; // 当前角度-目标角度

	if (Delta_Yaw <= -4096) // 小于-4096时，从4096开始自减
	{
		Delta_Yaw += 8192;
	}
	else if (Delta_Yaw >= 4096) // 大于4096时，从-4096开始自增
	{
		Delta_Yaw -= 8192;
	}

	static uint8_t time = 5; // 控制周期计时器，用于降低外环PID更新频率
	if (Delta_Yaw < 10 && Delta_Yaw > -10)
	{
		Delta_Yaw = 0;
	}
	if (time >= kk)
	{
		M6020s_Yaw.targetSpeed = Position_PID(&M6020s_YawOPID, 0, Delta_Yaw);
		time = 0;
	}
	M6020s_Yaw.outCurrent = Position_PID_Yaw(&AutoAim_M6020s_YawIPID, &FuzzyPID_AimYaw, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
	M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
	time++;
}

/**
 * @brief  M6020s_Yaw云台相关代码
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Sport_Out_Little_Yaw(void) // 先算出M6020s_Yaw.outCurrent，再赋值给data，通过CAN_SendData发送
{
	/***********************************/
	if (ControlMes.modelFlag == model_Record)
	{
		M6020s_Yaw.InfoUpdateFrame = 0; // 检录状态下，将yaw轴在线状态置零
		return;
	}
	else if (M6020s_Yaw.InfoUpdateFlag == 1)
	{
		Cloud_Yaw_Angle_Set(); // 正常在线状态下，设置yaw轴6020电机角度
	}
	else
	{
		return;
	}
}


/**
 * @brief  云台电机输出
 * @param  void
 * @retval void
 * @attention
 */
void Cloud_Sport_Out(void)
{
	Cloud_Pitch_Angle_Set();	  // pitch轴控制
	Cloud_Sport_Out_Little_Yaw(); // yaw轴控制
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
