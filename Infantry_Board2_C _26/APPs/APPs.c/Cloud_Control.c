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
positionpid_t M6020s_YawIPID;
positionpid_t M6020s_Yaw_SpeedPID;
positionpid_t M6020s_YawOPID;
positionpid_t AutoAim_M6020s_YawIPID;
positionpid_t AutoAim_M6020s_YawOPID;
/************电机PID END***********/

/****************卡拉曼滤波结构体创建*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman_manul;
/****************卡拉曼滤波结构体创建 End*****************/

/********变量声明********/
Cloud_t Cloud;
float Control_Self_Yaw;
float shit;
/********数据声明********/
float Linear=2.75f;
float Setup_Angleoffset = -3000;
uint8_t kk =8;
/********全局变量声明********/
extern M6020s_t* M6020_Array[1]; //对应电机的ID必须为：索引+1
extern Saber_Angle_t Saber_Angle;

/********函数声明********/
void Cloud_Init(void);
void Cloud_Yaw_Angle_Set(void);
void Cloud_Sport_Out(void);
void Cloud_Self_Yaw(void);
void PID_Clear_Yaw(void);

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
	Cloud.Target_Yaw = M6020s_Yaw.realAngle + Saber_Angle.Yaw /360.0f *  8192.0f; 

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
	Clear_PositionPIDData(&M6020s_YawIPID);
	Clear_PositionPIDData(&M6020s_YawOPID);
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
	if(M6020s_Yaw.InfoUpdateFrame <= 30)
	{
		Cloud.Target_Yaw = Saber_Angle.Yaw /360.0f *  8192.0f + M6020s_Yaw.realAngle ;
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
	float Angle_Yaw_Real = Saber_Angle.Yaw /360.0f *  8192.0f ;/* 8192/360*/  //真实角度
	float Angle_Err_Yaw = M6020s_Yaw.realAngle + Angle_Yaw_Real;	          //角度差值
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
				M6020s_Yaw.targetSpeed = Position_PID(&M6020s_YawOPID,  0 ,Delta_Yaw);	
				time = 0;
			}
			M6020s_Yaw.outCurrent = Position_PID_Yaw(&M6020s_YawIPID, &FuzzyPID_Yaw, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
			M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman_manul, M6020s_Yaw.outCurrent);
			time ++;
	}
	else if(ControlMes.AutoAimFlag==1)
	{
		M6020s_Yaw.targetSpeed = Position_PID(&AutoAim_M6020s_YawOPID,  0 ,Delta_Yaw);	
    M6020s_Yaw.outCurrent = Position_PID_Yaw(&AutoAim_M6020s_YawIPID, &FuzzyPID_AimYaw, M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
		M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
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
			M6020s_Yaw.InfoUpdateFrame = 0;
			return;
		}
		else if(M6020s_Yaw.InfoUpdateFlag == 1)
		{
			Cloud_FUN.Cloud_Yaw_Angle_Set();
		}
		else
		{
			return;
		}

	uint8_t data[8] = { 0 };
	
	/**********传递Yaw编码器数值**********/
	float Angle_Cloud = M6020s_Yaw.realAngle;
		Angle_Cloud = M6020s_Yaw.realAngle +Setup_Angleoffset;
		if(Angle_Cloud > 4096)
		{
			Angle_Cloud -= 8192;
		}
	Omni_Fun.Omni_GetAngle(-1*Angle_Cloud/8192.0f*360);

	/***************************将电流参数发送给电机*******************************/
	M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent, 0, 0, 0, data);
	Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M6020_SENDID, data);
}

