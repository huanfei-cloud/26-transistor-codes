 /**
 * @file Task_RobotControl.c
 * @author ZS
 * @brief
 * @version 0.1
 * @date 2025-03-3
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "SBUS.h"
#include "DT7.h"

void DT7_Control(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(5); //每5毫秒强制进入数据发送
    for (;;)
    {		
				DT7_Handle(); //处理大疆遥控器数据
   			vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

