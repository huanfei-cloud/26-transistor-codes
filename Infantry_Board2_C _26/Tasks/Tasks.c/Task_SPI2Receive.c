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
    const TickType_t TimeIncrement = pdMS_TO_TICKS(10); //每100毫秒强制进入
	 for( ; ; )
	 {
		 //验证是否设置菊花链成功
			MA600_flag[1] = MA600_Read_Reg(&MA600s[0],MA600_Reg_DAISY_RWM);
		 //清除错误标识
		 MA600_Clear_ErrFlags(&MA600s[0]);
		 //读取单个编码器数据
		  MA600_Get_Angle(&MA600s[0]);	
//		 //菊花链模式读取数据
//		 MA600s_Read_DaisyChain(); 
		 
		 
		  
			
		 vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	 }
 }
 