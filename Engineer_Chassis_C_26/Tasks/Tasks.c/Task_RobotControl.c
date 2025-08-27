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
				Board2_FUN.Board2_To_1();
        DT7_Handle();
				PowerControl_Fun.PowerControl_MsgSend();
				Steer_Omni_Chassis_Out();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
