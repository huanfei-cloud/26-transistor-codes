/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright
 *
 */
#include "Cloud_Control.h"

/************电机PID***********/
positionpid_t J6006s_YawIPID;
positionpid_t J6006s_YawOPID;
positionpid_t AutoAim_J6006s_YawIPID;
positionpid_t AutoAim_J6006s_YawOPID;
/************电机PID END***********/

/****************卡拉曼滤波结构体创建*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman_manul;
/****************卡拉曼滤波结构体创建 End*****************/

Cloud_t Cloud;
extern Saber_Angle_t Saber_Angle;
float Control_Self_Yaw;

float shit;
uint8_t kk =8;
float Linear=2.75f;
float Setup_Angleoffset = -3000;

/********数据声明********/
//MIT模式数据初始化设置
float J6006_Yaw_T = 1.0f; // 云台yaw轴所需扭矩
float J6006_Yaw_P = 0.0f; 
float J6006_Yaw_V = 0.0f;
float J6006_Yaw_Kp = 0.0f; //目标位置给零且KP给零直接转为速度模式
float J6006_Yaw_Kd = 1.0f;
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
	//电机使能函数在掉线检测任务里写着
	
	//MIT模式控制数据初始化
	DM_MIT_Init(&J6006s_Yaw,J6006_Yaw_T,J6006_Yaw_P,J6006_Yaw_V,J6006_Yaw_Kp,J6006_Yaw_Kd);

	//保存启动时刻的机械角度
	Cloud.Target_Yaw = J6006s_Yaw.realAngle + Saber_Angle.Yaw /360.0f *  8192.0f; 

	One_Kalman_Create(&Cloud_YawMotorAngle_Error_Kalman, 1, 10);
	One_Kalman_Create(&Cloud_YAWODKalman, 1, 10);
	One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 6);
	One_Kalman_Create(&Cloud_YawCurrent_Kalman_manul, 1, 6);
}




/**
  * @brief  M6020_Yaw电机PID清除
  * @param  void
  * @retval void
  * @attention
  */
void PID_Clear_Yaw(void)
{
	Clear_PositionPIDData(&J6006s_YawOPID);
	Clear_PositionPIDData(&AutoAim_J6006s_YawOPID);
}


/**
  * @brief  M6020_Yaw电机角度调整（陀螺仪），修正电机电流数据
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Yaw_Angle_Set(void)
{
	/**************************云台Yaw6020电机双环控制计算*****************************/
	if(J6006s_Yaw.InfoUpdateFrame <= 30)
	{
		Cloud.Target_Yaw = Saber_Angle.Yaw /360.0f *  8192.0f + J6006s_Yaw.realAngle ;
	}
	static uint8_t time=5;

	if (Cloud.Target_Yaw > 8192)
	{
		Cloud.Target_Yaw -= 8192;
	}
	else if (Cloud.Target_Yaw < 0)
	{
		Cloud.Target_Yaw += 8192;
	}

	/**************************Yaw轴电机控制，遥控器数据映射到位置角度*****************/
	float Angle_Yaw_Real = Saber_Angle.Yaw /360.0f *  8192.0f ;/* 8192/360*/  //地盘yaw世界坐标系的角度
	float Angle_Err_Yaw = J6006s_Yaw.realAngle + Angle_Yaw_Real;	          //云台yaw世界坐标系的角度
	/*Err值为-4096 ~ 8192+4096，Target为 0 ~ 8191，第一次调整Err为 -4096 ~ 4096 */
	/*解决跨圈问题*/
	if (Angle_Err_Yaw > 4096 )
	{
		Angle_Err_Yaw -= 8192 ;
	}
	ControlMes.yaw_realAngle = Angle_Err_Yaw;
	
	//Gimbal_Chassis_Pitch_Translate();    //云台相对底盘pitch轴角度赋值函数
	
	float Delta_Yaw = Angle_Err_Yaw - Cloud.Target_Yaw + Linear*Saber_Angle.Z_Vel  ;
	
	
	/*Derta的值 -4096-8191 ~ 4096*/
	if ( Delta_Yaw <=  -4096)
	{
		Delta_Yaw += 8192 ;
	}


	/*外环、内环，目标角度差，计算电流并滤波*/ /*Target_xxx为控制值*/

	if(ControlMes.AutoAimFlag==0)
	{
					/*死区*/
			if(Delta_Yaw < 5 && Delta_Yaw > -5)
			{
				Delta_Yaw = 0;
			}
			/*角度差值滤波*/
		  Delta_Yaw = One_Kalman_Filter(&Cloud_YawMotorAngle_Error_Kalman, Delta_Yaw);
			if( time >= kk )
			{
				J6006s_Yaw.outSpeed = Position_PID(&J6006s_YawOPID,  0 ,Delta_Yaw);	
				time = 0;
			}
			J6006s_Yaw.outTorque = Position_PID_Yaw(&J6006s_YawIPID, &FuzzyPID_Yaw, J6006s_Yaw.outSpeed, J6006s_Yaw.realSpeed);
			J6006s_Yaw.outTorque = One_Kalman_Filter(&Cloud_YawCurrent_Kalman_manul, J6006s_Yaw.outTorque);
			time ++;
	}
	else if(ControlMes.AutoAimFlag==1)
	{
		  J6006s_Yaw.outSpeed = Position_PID(&AutoAim_J6006s_YawOPID,  0 ,Delta_Yaw);	
      J6006s_Yaw.outTorque = Position_PID_Yaw(&AutoAim_J6006s_YawIPID, &FuzzyPID_AimYaw, J6006s_Yaw.outSpeed, J6006s_Yaw.realSpeed);
		  J6006s_Yaw.outTorque = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, J6006s_Yaw.outTorque);
	}
}


/**
  * @brief  M6020电机输出
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Sport_Out(void)
{
	    /**********电流参数的设置**********/
		if(ControlMes.modelFlag == model_Record)
		{
			J6006s_Yaw.InfoUpdateFrame = 0;
			return;
		}
		else if(J6006s_Yaw.InfoUpdateFlag == 1)
		{
			Cloud_FUN.Cloud_Yaw_Angle_Set();
		}
		else
		{
			return;
		}
	
	/**********传递Yaw编码器数值**********/
	  float Angle_Cloud = J6006s_Yaw.realAngle;
		Angle_Cloud = J6006s_Yaw.realAngle +Setup_Angleoffset;
		if(Angle_Cloud > 4096)
		{
			Angle_Cloud -= 8192;
		}
	Steer_Omni_GetAngle(-1*Angle_Cloud/8192.0f*360);

	/***************************将电流参数发送给电机*******************************/
		uint8_t data[8] = {0};
		DM_setParameter(J6006s_Yaw.outPosition, J6006s_Yaw.outSpeed, J6006s_Yaw.outKp, J6006s_Yaw.outKd, J6006s_Yaw.outTorque,J6006_MaxP,J6006_MaxV,J6006_MaxT, data);
		Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, J6006_SENDID_Yaw, data);
}

