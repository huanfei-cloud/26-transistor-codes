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


float data[6]={0};
uint16_t _num;

void VOFA_Handle(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(15);
    for (;;)
    {
        data[0]=M3508_Array[Fric_Front_1].realSpeed;
        data[1]=M3508_Array[Fric_Front_2].realSpeed;
        data[2]=M3508_Array[Fric_Front_3].realSpeed;
        data[3]=M3508_Array[Fric_Back_1].realSpeed;
        data[4]=M3508_Array[Fric_Back_2].realSpeed;
        data[5]=M3508_Array[Fric_Back_3].realSpeed;
        JustFloat_Send(data,6,USART1);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
