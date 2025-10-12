/**
 * @file Task_RobotControl.c
 * @author rm_control_team
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_RobotControl.h"


void Robot_Control(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ������ݷ���
	
//	while(!M6020s_Yaw.InfoUpdateFlag)
//	{
//		vTaskDelay(1);
//	}
//	Cloud_FUN.Cloud_Init();

    for (;;)
    {
//        HAL_UART_Transmit(&huart6,"Task_RobotControl",sizeof("Task_RobotControl "),0xff);
//				DT7_Handle();
        Saber_Read();
        Cloud_FUN.Cloud_Sport_Out();
			
			/********���yaw���������򶶶�����********/
		  if( M6020s_Yaw.realAngle < 1900)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
				
			}
			else if( M6020s_Yaw.realAngle > 3700 && M6020s_Yaw.realAngle < 5700)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
				
			}
			else if( M6020s_Yaw.realAngle > 7900)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
			}
			else
			{
				M6020s_YawIPID.Kp = yaw_I_p;
				M6020s_YawIPID.Kd = yaw_I_d;
			}
			AutoAim_M6020s_YawIPID.Kp = 1100.0f;
			AutoAim_M6020s_YawIPID.Ki =  0.5f;
			AutoAim_M6020s_YawIPID.Kd = 950.0f;
			AutoAim_M6020s_YawIPID.Kf = 6.0f;
			AutoAim_M6020s_YawOPID.Kp = 0.16;
			AutoAim_M6020s_YawOPID.Ki = 0.00001;
			AutoAim_M6020s_YawOPID.Kd = 0.01;
			AutoAim_M6020s_YawOPID.Kf = yaw_O_Aim_f;
			
			if( M6020s_Yaw.realAngle > 5250 && M6020s_Yaw.realAngle < 5600)
			{
			AutoAim_M6020s_YawOPID.Kp = 0.08f;
			AutoAim_M6020s_YawOPID.Kd = 0.03f;
			}
			else if( M6020s_Yaw.realAngle > 4300 && M6020s_Yaw.realAngle < 4500)
			{
			AutoAim_M6020s_YawOPID.Kp = 0.08f;
			AutoAim_M6020s_YawOPID.Kd = 0.03f;
			}
			else if( M6020s_Yaw.realAngle > 1600 && M6020s_Yaw.realAngle < 1800)
			{
			AutoAim_M6020s_YawOPID.Kp = 0.08f;
			AutoAim_M6020s_YawOPID.Kd = 0.03f;
			}
			else if( M6020s_Yaw.realAngle > 300 && M6020s_Yaw.realAngle < 500)
			{
			AutoAim_M6020s_YawOPID.Kp = 0.08f;
			AutoAim_M6020s_YawOPID.Kd = 0.03f;
			}
			else if( M6020s_Yaw.realAngle > 8100 || M6020s_Yaw.realAngle < 100)
			{
			AutoAim_M6020s_YawOPID.Kp = 0.08f;
			AutoAim_M6020s_YawOPID.Kd = 0.03f;
			}
			else
			{
				AutoAim_M6020s_YawOPID.Kp = 0.10f;
			AutoAim_M6020s_YawOPID.Kd = 0.01f;
			}
			/******************End******************/
			  
			  Board2_FUN.Board2_To_1();
				PowerControl_Fun.PowerControl_MsgSend();
				steer_chassis_out();
			
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
