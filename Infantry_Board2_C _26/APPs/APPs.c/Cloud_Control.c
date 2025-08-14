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

/************���PID***********/
positionpid_t M6020s_YawIPID;
positionpid_t M6020s_Yaw_SpeedPID;
positionpid_t M6020s_YawOPID;
positionpid_t AutoAim_M6020s_YawIPID;
positionpid_t AutoAim_M6020s_YawOPID;
/************���PID END***********/

/****************�������˲��ṹ�崴��*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman;
One_Kalman_t Cloud_YawCurrent_Kalman_manul;
/****************�������˲��ṹ�崴�� End*****************/

/********��������********/
Cloud_t Cloud;
float Control_Self_Yaw;
float shit;
/********��������********/
float Linear=2.75f;
float Setup_Angleoffset = -3000;
uint8_t kk =8;
/********ȫ�ֱ�������********/
extern M6020s_t* M6020_Array[1]; //��Ӧ�����ID����Ϊ������+1
extern Saber_Angle_t Saber_Angle;

/********��������********/
void Cloud_Init(void);
void Cloud_Yaw_Angle_Set(void);
void Cloud_Sport_Out(void);
void Cloud_Self_Yaw(void);
void PID_Clear_Yaw(void);

/***************����ӿڶ���***************/
Cloud_FUN_t Cloud_FUN = Cloud_FUNGroundInit;
#undef Cloud_FUNGroundInit


/**
 * @brief  ��̨��ʼ�������ò�������λ��̨
 * @param  None
 * @retval None
 */
void Cloud_Init(void)
{

	//��������ʱ�̵Ļ�е�Ƕ�
	Cloud.Target_Yaw = M6020s_Yaw.realAngle + Saber_Angle.Yaw /360.0f *  8192.0f; 

	One_Kalman_Create(&Cloud_YawMotorAngle_Error_Kalman, 1, 10);
	One_Kalman_Create(&Cloud_YAWODKalman, 1, 10);
	One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 6);
	One_Kalman_Create(&Cloud_YawCurrent_Kalman_manul, 1, 6);
}


/**
  * @brief  M6020_Yaw���PID���
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
  * @brief  M6020_Yaw����Ƕȵ����������ǣ������������������
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Yaw_Angle_Set(void)
{
	/**************************��̨Yaw6020���˫�����Ƽ���*****************************/
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

	/**************************Yaw�������ƣ�ң��������ӳ�䵽λ�ýǶ�*****************/
	float Angle_Yaw_Real = Saber_Angle.Yaw /360.0f *  8192.0f ;/* 8192/360*/  //��ʵ�Ƕ�
	float Angle_Err_Yaw = M6020s_Yaw.realAngle + Angle_Yaw_Real;	          //�ǶȲ�ֵ
	/*ErrֵΪ-4096 ~ 8192+4096��TargetΪ 0 ~ 8191����һ�ε���ErrΪ -4096 ~ 4096 */
	/*�����Ȧ����*/
	if (Angle_Err_Yaw > 4096 )
	{
		Angle_Err_Yaw -= 8192 ;
	}
	ControlMes.yaw_realAngle = Angle_Err_Yaw;
	
	//Gimbal_Chassis_Pitch_Translate();    //��̨��Ե���pitch��Ƕȸ�ֵ����
	
	float Delta_Yaw = Angle_Err_Yaw - Cloud.Target_Yaw + Linear*Saber_Angle.Z_Vel  ;
	
	
	/*Derta��ֵ -4096-8191 ~ 4096*/
	if ( Delta_Yaw <=  -4096)
	{
		Delta_Yaw += 8192 ;
	}


	/*�⻷���ڻ���Ŀ��ǶȲ����������˲�*/ /*Target_xxxΪ����ֵ*/

	if(ControlMes.AutoAimFlag==0)
	{
					/*����*/
			if(Delta_Yaw < 5 && Delta_Yaw > -5)
			{
				Delta_Yaw = 0;
			}
			/*�ǶȲ�ֵ�˲�*/
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
  * @brief  M6020������
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Sport_Out(void)
{
	    /**********��������������**********/
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
	
	/**********����Yaw��������ֵ**********/
	float Angle_Cloud = M6020s_Yaw.realAngle;
		Angle_Cloud = M6020s_Yaw.realAngle +Setup_Angleoffset;
		if(Angle_Cloud > 4096)
		{
			Angle_Cloud -= 8192;
		}
	Omni_Fun.Omni_GetAngle(-1*Angle_Cloud/8192.0f*360);

	/***************************�������������͸����*******************************/
	M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent, 0, 0, 0, data);
	Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M6020_SENDID, data);
}

