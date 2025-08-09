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
	 for( ; ; )
	 {
		 //菊花链模式读取数据
		 MA600s_Read_DaisyChain();
//		 //读取单个编码器数据
//		MA600_Get_Angle(&MA600s[0]);	 
//		 HAL_Delay(10);
	 }
 }
 