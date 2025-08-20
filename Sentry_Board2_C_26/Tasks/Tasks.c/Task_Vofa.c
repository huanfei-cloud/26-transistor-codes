/**
 * @file Task_Vofa.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-05-30
 *
 */
#include "Task_Vofa.h"
#include "vofa.h"
#include "M3508_Motor.h"

/**
  * @Data   2025-05-06
  * @brief  SPI2接收任务
  * @param  void
  * @retval void
  */
void Vofa_Assist(void const *argument)
{
	 for( ; ; )
	 {
		 portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10); //每100毫秒强制进入
	 for( ; ; )
	 {
		 float data = M6020s_Yaw.realAngle;
     JustFloat_Send(&data,1,USART6);
		 
		 vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	 }
	 }
}
