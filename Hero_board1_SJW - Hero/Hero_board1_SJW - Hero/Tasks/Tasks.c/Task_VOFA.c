/**
 * @file Task_VOFA.c
 * @author SJW
 * @brief 使用时将data和num替换为实际数据
 * @version 0.1
 * @date 2025-08-9
 *
 * @copyright Copyright (c)
 *
 */

#include "Task_VOFA.h"


float data;
uint16_t _num;

void VOFA_Handle(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(15);
    for (;;)
    {
        JustFloat_Send(&data,_num,USART1);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
