/**
 * @file Task_SPI2.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-05-06
 *
 */
#include "Task_SPI2Receive.h"
#include "MA600_use.h"

/**
  * @Data   2025-05-06
  * @brief  SPI2接收任务
  * @param  void
  * @retval void
  */
 void SPI2Receives(void const *argument)
 {
	 portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10); //每10毫秒强制进入
	 for( ; ; )
	 {
		 	
		
		 vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	 }
 }
 