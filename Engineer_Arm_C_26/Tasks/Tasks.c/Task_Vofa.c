/**
 * @file Task_Vofa.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-05-30
 *
 */
#include "Task_Vofa.h"


/**
  * @Data   2025-05-06
  * @brief  Vofa发送任务
  * @param  void
  * @retval void
  */
 void Vofa_Assist(void const *argument)
 {
	 portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10); //每100毫秒强制进入
	 for( ; ; )
	 {
		 
		 
		 vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	 }
}
