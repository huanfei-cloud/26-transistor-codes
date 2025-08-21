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
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入数据发送
	
//	while(!M6020s_Yaw.InfoUpdateFlag)
//	{
//		vTaskDelay(1);
//	}
//	Cloud_FUN.Cloud_Init();

    for (;;)
    {
//        HAL_UART_Transmit(&huart6,"Task_RobotControl",sizeof("Task_RobotControl "),0xff);
        Saber_Read();
        Cloud_FUN.Cloud_Sport_Out();
			
			/********解决yaw轴两个区域抖动问题********/
		  if( M6020s_Yaw.realAngle < 1900)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
				
				if( M6020s_Yaw.realAngle > 700 && M6020s_Yaw.realAngle < 900 )
				{
			      AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p - 300.0f;
				    AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d - 300.0f;
				}
				else
				{
					  AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p - 200.0f;
				    AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d - 200.0f;
				}
			}
			else if( M6020s_Yaw.realAngle > 3700 && M6020s_Yaw.realAngle < 5700)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
				
				if( M6020s_Yaw.realAngle > 4500 && M6020s_Yaw.realAngle < 4700)
				{
				   AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p - 300.0f;
				   AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d - 300.0f;
				}
				else
				{
					AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p - 200.0f;
				  AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d - 200.0f;
				}
			}
			else if( M6020s_Yaw.realAngle > 7900)
			{
				M6020s_YawIPID.Kp = yaw_I_p - 150.0f;
				M6020s_YawIPID.Kd = yaw_I_d - 150.0f;
				AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p - 200.0f;
				AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d - 200.0f;
			}
			else
			{
				M6020s_YawIPID.Kp = yaw_I_p;
				M6020s_YawIPID.Kd = yaw_I_d;
				AutoAim_M6020s_YawIPID.Kp = yaw_I_Aim_p;
				AutoAim_M6020s_YawIPID.Kd = yaw_I_Aim_d;
			}
			/******************End******************/
			  
        DT7_Handle();
			  Board2_FUN.Board2_To_1();
				PowerControl_Fun.PowerControl_MsgSend();
				steer_chassis_out();
			
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
