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
#include "Shoot.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"
#include "SBUS.h"
#include "DT7.h"

#include "BSP_Fdcan.h"
#include "BSP_Test.h"
void Robot_Control(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每3毫秒强制进入数据发送
    for (;;)
    {
			
			Cloud_FUN.Remote_Change();     //变速小陀螺
			Cloud_FUN.Cloud_Sport_Out();   //电流数据统一在shoot里面发，先算cloud的数据
			Shoot_Fun.Shoot_Processing();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
