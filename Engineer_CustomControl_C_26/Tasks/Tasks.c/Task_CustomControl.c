/**
 * @file Task_Vofa.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-05-30
 *
 */
#include "Task_CustomControl.h"

/**
  * @Data   2025-08-21
  * @brief  机械臂任务执行
  * @param  void
  * @retval void
  */
void Custom_Control(void const *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入
	for( ; ; )
	{
		
		CustomControl_Set_Location();
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
