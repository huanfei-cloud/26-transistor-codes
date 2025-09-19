/**
 * @file Task_Arm.c
 * @author Transistor
 * @brief 工程机械臂的任务函数
 * @version 1.2
 * @date 2025-08-15
 * @copyright Transistor BUAA
 */

#include "Task_Arm.h" 

void Arm_Control(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(7); // 每2毫秒强制进入数据发送

    for (;;)
    {        
        DM_Motors_Out();
			  DJ_Motors_Out();

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
