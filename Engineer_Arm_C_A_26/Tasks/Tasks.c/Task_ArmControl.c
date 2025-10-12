/**
 * @file Task_Vofa.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-05-30
 *
 */
#include "Task_ArmControl.h"

/**
  * @Data   2025-08-21
  * @brief  机械臂任务执行
  * @param  void
  * @retval void
  */
void Arm_Task(void const *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入
	for( ; ; )
	{
		
    DT7_Handle(); 
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
