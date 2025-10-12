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
	//如果数据来自USART1,即为IMU数据
	if(huart->Instance == USART1)
	{
		if(N100_Rxbuffer[0] == FRAME_HEAD && N100_Rxbuffer[55] == FRAME_END)
		{
			N100_Read();
		}
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart1,N100_Rxbuffer,sizeof(N100_Rxbuffer));
	}
}
