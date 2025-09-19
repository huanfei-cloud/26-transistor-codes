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

#include "OffLineCheck.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

//static uint16_t yaw_Frame = 0;

void Off_Line_Check(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100); //每100毫秒强制进入
    for (;;)
    {
								
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
