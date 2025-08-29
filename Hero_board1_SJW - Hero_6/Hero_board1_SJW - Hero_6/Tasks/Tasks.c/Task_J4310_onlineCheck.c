/**
 * @file Task_J4310_onlineCheck.c
 * @author SJW
 * @brief 检测4340电机掉线并使能,由于达秒电机控制方式均相同，故采用4310的检测方式
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c)
 *
 */

#include "Task_J4310_onlineCheck.h"


static uint16_t pitch_Frame = 0;

void J4310_onlineCheck(void const *argument)
{

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    for (;;)
    {
        if (pitch_Frame == J4340s_Pitch.InfoUpdateFrame)
        {
            J4340_Fun.J4340_Enable();
        }
        pitch_Frame = J4340s_Pitch.InfoUpdateFrame;
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
