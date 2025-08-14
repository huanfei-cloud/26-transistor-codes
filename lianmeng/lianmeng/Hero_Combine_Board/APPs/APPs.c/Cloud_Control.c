/**
 * @file Cloud_control.c
 * @author Cyx,ZS
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright
 *
 */
#include "Cloud_Control.h"
#include "FeedForward.h"
#include "M6020_Motor.h"
#include "BSP_Fdcan.h"
#include "Extern_Handles.h"
#include "Saber_C3.h"

Cloud_t Cloud;

/************电机PID***********/
Struct_PID_Manage_Object_Fuzzy M6020s_YawIPID;
Struct_PID_Manage_Object_Fuzzy M6020s_Yaw_SpeedPID;
Struct_PID_Manage_Object_Fuzzy M6020s_PitchIPID;
Struct_PID_Manage_Object_Fuzzy M6020s_YawOPID;
Struct_PID_Manage_Object_Fuzzy M6020s_PitchOPID;
Struct_PID_Manage_Object_Fuzzy M6020s_AimYawIPID;
Struct_PID_Manage_Object_Fuzzy M6020s_AimYawOPID;
/************电机PID END***********/

/****************Pithch限位*****************/
//const float Cloud_Pitch_Min = 700;        //Pitch反向打
const float Cloud_Pitch_Min = -200;    //
const float Cloud_Pitch_Max = 60;
const float Pitch_Angle_Init = 0;   //pitch轴水平是270
const float Cloud_Pitch_Center = 0;
const float Cloud_Pitch_Derta =Cloud_Pitch_Center-Cloud_Pitch_Min ;
/****************Pithch限位  End*****************/


/****************卡尔曼滤波结构体创建*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;
One_Kalman_t Cloud_PitchMotorAngle_Error_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman;
One_Kalman_t Cloud_PitchCurrent_Kalman;
/****************卡拉曼滤波结构体创建 End*****************/

Cloud_t Cloud;
extern M6020s_t* M6020_Array[2]; //对应电机的ID必须为：索引+1
extern Saber_Angle_t Saber_Angle;
float Control_Self_Yaw;
float Control_Self_Pitch;

float Linear=-5.1f;
uint8_t time_flag = 5;
float Pitch_Torque = 3.f;     //云台所需扭矩（额定扭矩）
float Pitch_v = 2;
float Pitch_Kp = 90;
float Pitch_Kd = 2;
float Pitch_RC_Sen = 0.0003;
float Yaw_RC_Sen = 0.005;
int16_t Aim_Yaw_RawAngle;

int Aim_Flag = 0;
float offest = 0;
int16_t Cloud_Aim_Yaw_Flag;
int16_t Cloud_Aim_Pitch_Flag;
int16_t Cloud_Manual_Yaw_Flag;
int16_t Cloud_Manual_Pitch_Flag;

void Cloud_Init(void);
void Cloud_Yaw_Angle_Set(void);
void Cloud_Pitch_Angle_Set(void);
void Cloud_Sport_Out(void);
void Cloud_Self_Yaw(void);
void Cloud_Self_Pitch(void);
void FeedForward_Yaw(void);
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

	//保存启动时刻的机械角度
	//Cloud.Target_Yaw = M6020s_Yaw.realAngle; //开机让云台回中。
	Cloud_Manual_Yaw_Flag = Cloud_Pitch_Center;
	
	//Pitch_Angle_Init = 1300;
	Cloud_Manual_Pitch_Flag = Cloud.Target_Pitch;

//	M6020_Fun.M6020_setTargetAngle(&M6020s_Yaw, Cloud.Target_Yaw);
//	M6020_Fun.M6020_setTargetAngle(&M6020s_Pitch, Cloud_Pitch_Center);

	One_Kalman_Create(&Cloud_YawMotorAngle_Error_Kalman, 1, 40);
	One_Kalman_Create(&Cloud_PitchMotorAngle_Error_Kalman, 1.5, 40);
	One_Kalman_Create(&Cloud_YAWODKalman, 1, 10);
	One_Kalman_Create(&Cloud_PITCHODKalman, 1, 10);
	One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 40);
	One_Kalman_Create(&Cloud_PitchCurrent_Kalman, 3, 10);
}

/**
 * @brief  通过6020机械角度的方式获取云台Yaw旋转的角度（偏移车正前方的角度-中心点）
 * @param[in]  None
 * @retval 360度的角度值。
 */
float Cloud_GetYawAngleWithCenter(void)
{
	return (M6020s_Yaw.totalAngle - Cloud_Yaw_Center) / M6020_mAngleRatio;
}

/**
 * @brief  强制设置云台机械坐标（绕过缓冲区）
* @param[in]  	posYaw:左右
*				posPitch:俯仰
 * @retval None
 */
void Cloud_setAbsPosForced(float posYaw, float posPitch)
{
	Cloud.Target_Yaw = posYaw;
	Cloud.Target_Pitch = posPitch;
}

/**
 * @brief 清除云台电机PID缓冲值
 *
 * @return
 */
//static void Cloud_ClearMoterPIDData(void)
//{
//	/*************清除Yaw轴****************/
////    Clear_PositionPIDData(&M6020s_YawOPID);
////    Clear_PositionPIDData(&M6020s_YawIPID);



/**
  * @brief  M6020_Yaw电机角度调整（陀螺仪），修正电机电流数据
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Yaw_Angle_Set(void)
{
	/**************************云台Yaw6020电机双环控制计算*****************************/
	static uint8_t time=5;
	static float Delta_Yaw =0;
	static float Delta_Yaw_DT7=0;

	if (ControlMes.AutoAimFlag == 1)
	{
		Delta_Yaw_DT7 += (float)ControlMes.yaw_velocity * Yaw_RC_Sen;
		Cloud.Target_Yaw = Auto_Aim_Yaw + Delta_Yaw_DT7;
		Cloud_Manual_Yaw_Flag = Cloud.Target_Yaw;
		Aim_Flag = 1;
	}
		
	else
	{
		Delta_Yaw += (float)ControlMes.yaw_velocity * Yaw_RC_Sen;

		if(M6020s_Yaw.InfoUpdateFrame <= 30 && M6020s_Yaw.InfoUpdateFlag!=1)
		{
			Cloud.Target_Yaw = M6020s_Yaw.realAngle;//IMU_Angle.Yaw + M6020s_Yaw.realAngle ;
		}
		Cloud.Target_Yaw += (float)ControlMes.yaw_velocity * Yaw_RC_Sen;



	}
  /******************Yaw轴跨圈问题******************/
	if (Cloud.Target_Yaw > 8192)
	{
		Cloud.Target_Yaw -= 8192;
	}
	else if (Cloud.Target_Yaw < 0)
	{
		Cloud.Target_Yaw += 8192;
	}
	
	/**************************Yaw轴电机控制，遥控器数据映射到位置角度*****************/
	float Angle_Yaw_Real = -IMU_Angle.Yaw ;/* 8192/360*/	//真实角度
	float Angle_Err_Yaw = M6020s_Yaw.realAngle + Angle_Yaw_Real;	//角度差值
	Cloud.Yaw_Raw = Angle_Err_Yaw;
//	
	/*Err值为-4096 ~ 8192+4096，Target为 0 ~ 8191，第一次调整Err为 -4096 ~ 4096 */
	/*解决跨圈问题*/
	if (Angle_Err_Yaw > 4096 )
	{
		Angle_Err_Yaw -= 8192 ;
	}
	
	Aim_Yaw_RawAngle = Angle_Err_Yaw;

	float Derta_Yaw = Angle_Err_Yaw - Cloud.Target_Yaw + Linear*IMU_Angle.Z_Vel;
	
	/*Derta的值 -4096-8191 ~ 4096*/
	if ( Derta_Yaw <=  -4096)
	{
		Derta_Yaw += 8192 ;
	}

	/*外环、内环，目标角度差，计算电流并滤波*/ /*Target_xxx为控制值*/
 if(ControlMes.AutoAimFlag == 0)
 {
//	 	/*死区*/
//		if(Derta_Yaw < 1 &&Derta_Yaw > -1)
//		{
//			Derta_Yaw=0;
//		}
	 /*角度差值滤波*/
	 Derta_Yaw = One_Kalman_Filter(&Cloud_YawMotorAngle_Error_Kalman, Derta_Yaw);
	if(time>time_flag)
	{
		M6020s_Yaw.targetSpeed = PID_Model4_Update(&M6020s_YawOPID, &M6020s_YawO_FuzzyPID, 0, Derta_Yaw);
		time=0;
	}
	 M6020s_Yaw.outCurrent = PID_Model4_Update(&M6020s_YawIPID, &M6020s_YawI_FuzzyPID, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
	time ++;
 }
 else if(ControlMes.AutoAimFlag == 1)
 {
	 M6020s_Yaw.targetSpeed = PID_Model4_Update(&M6020s_AimYawOPID, &M6020s_AimYawO_FuzzyPID, 0, Derta_Yaw);
	 M6020s_Yaw.outCurrent = PID_Model4_Update(&M6020s_AimYawIPID, &M6020s_AimYawI_FuzzyPID, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
 }
 
	M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
	//FeedForward_Yaw();
	
}


/**
  * @brief  J4310_Pitch电机角度调整（陀螺仪），修正电机电流数据
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Pitch_Angle_Set(void)
{
    /****************************云台PitchJ4310电机******************************/
    /******************************遥控器数值传递******************************/
		static float Delta_Pitch =0;
		if (ControlMes.AutoAimFlag==1)
		{
			Delta_Pitch += (float)ControlMes.pitch_velocity * Pitch_RC_Sen;
			Cloud.Target_Pitch = Pitch_Angle_Init + Cloud.AutoAim_Pitch + Delta_Pitch;
			Aim_Flag = 1;
		}
			
		else
		{
			Delta_Pitch += (float)ControlMes.pitch_velocity * Pitch_RC_Sen;
			if(Aim_Flag == 0)
			{
				 Cloud.Target_Pitch = Delta_Pitch + Pitch_Angle_Init;
			}
			else if(Aim_Flag == 1)
			{
				 Cloud.Target_Pitch = Delta_Pitch + Pitch_Angle_Init;
			}

		}
		//if(Delta_Pitch>Delta_Pitch+
	  
    /**********Pitch轴限位**********/
    if (Cloud.Target_Pitch > Cloud_Pitch_Max)
    {
        Cloud.Target_Pitch = Cloud_Pitch_Max ;
    }
    else if (Cloud.Target_Pitch < Cloud_Pitch_Min)
    {
        Cloud.Target_Pitch = Cloud_Pitch_Min ;
    }

    /**************************Pitch轴电机控制，达妙J4310电机MIT模式，参数赋值*****************/
		J4310s_Pitch.outKp = Pitch_Kp;
		J4310s_Pitch.outKd = Pitch_Kd;
		J4310s_Pitch.outSpeed = Pitch_v;
		J4310s_Pitch.outTorque = Pitch_Torque;
    J4310s_Pitch.outPosition = Cloud.Target_Pitch;//One_Kalman_Filter(&Cloud_PitchCurrent_Kalman, Cloud.Target_Pitch);

}



/**
  * @brief  M6020电机输出
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Sport_Out(void)
{
//#if(USING_BOARD == BOARD1)
	
	/**********Pitch轴电流参数的设置**********/
	Cloud_Pitch_Angle_Set();
	//摩擦轮和拨弹电机的控制帧ID均为0x01FF
	//所以在shoot.c的Shoot_Processing函数里面统一发送这四个电机的电流

//#elif(USING_BOARD == BOARD2)
	uint8_t data[8] = { 0 };

	/**********Yaw轴电流参数的设置**********/
	Cloud_Yaw_Angle_Set();       
	//在Task_RobotControl里统一将电流参数发送给电机

//	/***************************将电流参数发送给电机*******************************/
	//M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent, 0, 0, 0, data);
	//Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan1, FDCAN_STANDARD_ID, M6020_SENDID, data);
//#endif
}
