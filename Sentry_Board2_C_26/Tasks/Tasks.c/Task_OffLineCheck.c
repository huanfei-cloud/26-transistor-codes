 /**
 * @file Task_OffLineCheck.c
 * @author Cyx
 * @brief
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_OffLineCheck.h"
#include "Cloud_Control.h"
#include "Extern_Handles.h"

static uint16_t yaw_Frame = 0;

void Off_Line_Check(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100); //每100毫秒强制进入
    for (;;)
    {
			//yaw掉线检测
				if(yaw_Frame == J6006s_Yaw.InfoUpdateFrame)
				{
					DM_Enable(0x05,&hcan1);
					
					J6006s_Yaw.InfoUpdateFlag = 0;
					J6006s_Yaw.InfoUpdateFrame = 0;
				}
				yaw_Frame = J6006s_Yaw.InfoUpdateFrame ;
				
				
								
		
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
