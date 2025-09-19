#include "BSP_Usart.h"

/***************用户数据声明****************/
/******************接口声明*****************/
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit

/**
 * @brief  接收空闲回调
 * @param  void
 * @retval void
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// FreeROTS退出中断时判断是否要进行任务切换
	//  如果数据来自USART3,即为遥控器数据
	if (huart->Instance == USART3)
	{
		// DT7遥控器
		DT7_RX_Finish = 1; // 已接受完一包数据
	}
}
