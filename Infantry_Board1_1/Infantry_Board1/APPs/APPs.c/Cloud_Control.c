/**
 * @file Cloud_control.c
 * @author Cyx��ZS
 * @brief
 * @version 2.5
 * @date 2023-08-15
 *
 * @copyright
 *
 */
#include "Cloud_Control.h"
#include "FeedForward.h"
#include "M6020_Motor.h"
#include "M3508_Motor.h"
#include "BSP_Fdcan.h"
#include "Extern_Handles.h"
#include "Saber_C3.h"
#include "FuzzyPID.h"

Cloud_t Cloud;

/************���PID***********/
positionpid_t M6020s_PitchIPID;
positionpid_t M6020s_PitchOPID;
positionpid_t AutoAim_M6020s_PitchIPID;
positionpid_t AutoAim_M6020s_PitchOPID;
/************���PID END***********/

/************��ͨ�˲�����***********/
static float filtered_angle_err_pitch = 0;  // ��һ���˲���ĽǶ����
float low_pass_filter_sen1=0.5;
float low_pass_filter_sen2=0.1;

float Cloud_Target_Aim_Flag;
float Cloud_Init_Angle;
float Pitch_Conpensate_Angle;     //Pitch��ǶȲ���ֵ
extern M6020s_t *M6020_Array[2]; //��Ӧ�����ID����Ϊ������+1
extern Saber_Angle_t Saber_Angle;

One_Kalman_t Cloud_PitchMotorAngle_Error_Kalman;
One_Kalman_t Cloud_PitchCurrent_Kalman;

void Cloud_Init(void);
void Cloud_Pitch_Angle_Set(void);
void Cloud_Sport_Out(void);
void PID_Clear_Pitch(void);
void Remote_Change(void);
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
    Cloud.Target_Pitch = Cloud_Pitch_Init;
	  Cloud.Pitch_Raw = Cloud_Pitch_Init ;
	  Cloud_Init_Angle = Cloud_Pitch_Init;
		Cloud.AutoAim_Pitch = 0;

    One_Kalman_Create(&Cloud_PitchMotorAngle_Error_Kalman, 1.5, 40);
    One_Kalman_Create(&Cloud_PITCHODKalman, 1, 10);
    One_Kalman_Create(&Cloud_PitchCurrent_Kalman, 6,10 );
		ControlMes.change_Flag = 0;
		ControlMes.shoot_Speed = 2;
		ControlMes.fric_Flag = 0;
		ControlMes.redial =0;
}


/**
  * @brief  M6020_Pitch���PID���
  * @param  void
  * @retval void
  * @attention
  */
void PID_Clear_Pitch(void)
{
	Position_PIDInit(&M6020s_PitchIPID, 75.0f, 5.0f, 0, 0.5, 20000, 10000, 10000);
	Position_PIDInit(&M6020s_PitchOPID, 0.24f, 0.00035, 0.9, 0.5, 20000, 10000, 10000);
}

/**
  * @brief  ��ͨ�˲�
  * @param  float float float �˲�������һ���˲����ֵ���˲�ϵ��
  * @retval void
  * @attention
  */
float low_pass_filter(float current_value, float prev_value, float alpha) 
{
    return alpha * current_value + (1 - alpha) * prev_value;
}


/**
  * @brief  M6020_Pitch����Ƕȵ��������������������
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Pitch_Angle_Set(void)
{
	static float AutoAim_Cloud_Delta = 0;
	static float Gimbal_Cloud_Delta = 0 ;
	
    /**************************��̨Pitch6020���˫�����Ƽ���*****************************/
    /******************************ң������ֵ����******************************/
	
	if (ControlMes.AutoAimFlag)
	{
		AutoAim_Cloud_Delta += (float)ControlMes.pitch_velocity * 0.006f;
		Cloud.Target_Pitch = Cloud_Init_Angle + Cloud.AutoAim_Pitch + AutoAim_Cloud_Delta ;
	}
	else if(ControlMes.AutoAimFlag==0)
	{
		Gimbal_Cloud_Delta += (float)ControlMes.pitch_velocity * 0.006f;		
		Cloud.Target_Pitch = Cloud_Init_Angle + Gimbal_Cloud_Delta ;

	}
	
	 /**********Gimbal_Cloud_Delta��λ**********/
	if (Gimbal_Cloud_Delta > Cloud_Pitch_Max - Cloud_Init_Angle)
	{
		 Gimbal_Cloud_Delta = Cloud_Pitch_Max - Cloud_Init_Angle ;
	}
	else if (Gimbal_Cloud_Delta < Cloud_Pitch_Min - Cloud_Init_Angle)
	{
		 Gimbal_Cloud_Delta = Cloud_Pitch_Min - Cloud_Init_Angle ;
	}
	
    /**********Pitch����λ**********/
    if (Cloud.Target_Pitch > Cloud_Pitch_Max)
    {
        Cloud.Target_Pitch = Cloud_Pitch_Max ;
    }
    else if (Cloud.Target_Pitch < Cloud_Pitch_Min)
    {
        Cloud.Target_Pitch = Cloud_Pitch_Min ;
    }

    /**************************Pitch�������ƣ�ң��������ӳ�䵽λ�ýǶ�*****************/
    float Angle_Err_Pitch = Cloud.Target_Pitch - M6020s_Pitch.realAngle ;  //�ǶȲ�ֵ

    /*�����Ȧ���⣨һ�㲻�ᣩ*/
    if (Angle_Err_Pitch > 4096)
    {
        Angle_Err_Pitch -= 8191;
    }
    else if (Angle_Err_Pitch < -4096)
    {
        Angle_Err_Pitch += 8191;
    }

    /*�ǶȲ�ֵ�˲�*/
    //Angle_Err_Pitch = One_Kalman_Filter(&Cloud_PitchMotorAngle_Error_Kalman, Angle_Err_Pitch);
    /*�⻷���ڻ���Ŀ��ǶȲ����������˲�*//*Target_xxxΪ����ֵ*/

    /*����*/
    if ( Angle_Err_Pitch > -1.0f &&  Angle_Err_Pitch < 1.0f)
    {
        Angle_Err_Pitch  = 0;
    }

	if(ControlMes.AutoAimFlag)
	{
    M6020s_Pitch.targetSpeed = Position_PID(&AutoAim_M6020s_PitchOPID, (int)Angle_Err_Pitch,0);
    M6020s_Pitch.outCurrent = Position_PID_Pitch(&AutoAim_M6020s_PitchIPID, &FuzzyPID_Pitch, M6020s_Pitch.targetSpeed, M6020s_Pitch.realSpeed);
    M6020s_Pitch.outCurrent = One_Kalman_Filter(&Cloud_PitchCurrent_Kalman, M6020s_Pitch.outCurrent);		
	}
  else
	{
    M6020s_Pitch.targetSpeed = Position_PID(&M6020s_PitchOPID, (int)Angle_Err_Pitch,0);
    M6020s_Pitch.outCurrent = Position_PID_Pitch(&M6020s_PitchIPID, &FuzzyPID_Pitch, M6020s_Pitch.targetSpeed, M6020s_Pitch.realSpeed);		
	}

    /*ǰ��*/
//    FeedForward_FUN.FeedForward_Yaw_Pitch();
		uint8_t data[8];

		M3508_FUN.M3508_setCurrent(M3508_Array[FricL_Wheel].outCurrent,
														 M3508_Array[FricR_Wheel].outCurrent,
														 M2006_Array[Dial_Motor].outCurrent,
														 M6020s_Pitch.outCurrent, data);

		Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan1, FDCAN_STANDARD_ID, SENDID_Fric_Dial, data);
}


/**
  * @brief  M6020������
  * @param  void
  * @retval void
  * @attention
  */
void Cloud_Sport_Out(void)
{
		if(M6020s_Pitch.InfoUpdateFrame <= 2 )
		{
			 Cloud.Target_Pitch = M6020s_Pitch.realAngle ;
		}
    /**********��������������**********/
		Cloud_FUN.Cloud_Pitch_Angle_Set();
}

/**
  * @brief  ����С����
  * @param  void
  * @retval void
  * @attention
  */

void Remote_Change(void)
{
	static int change_State = 0 ;
	static int change_Remote = 0 ;
	if(ControlMes.change_Flag == 1)
	{
		if(change_State == 0)
		{
			change_Remote += 3;
			ControlMes.z_rotation_velocity += 3;
			if(change_Remote >= 300)
			{
				change_State = 1;
			}
		}
		else if(change_State == 1)
		{
			change_Remote -= 3;
			ControlMes.z_rotation_velocity -= 3;
			if(change_Remote <= -300)
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

