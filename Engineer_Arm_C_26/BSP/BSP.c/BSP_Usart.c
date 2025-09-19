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
	// BaseType_t xHigherPriorityTask;        //FreeROTS退出中断时判断是否要进行任务切换
	// 如果数据来自USART3,即为遥控器数据
	if (huart->Instance == USART3)
	{
#if (RemoteControlMethod == TDF)
		{ // 天地飞遥控器
			memcpy(SBUS_RXBuffer, SBUS_Rx_Data, sizeof(SBUS_Rx_Data));
			SBUS_RX_Finish = 1; // 已接收完一包数据
		}
#elif (RemoteControlMethod == DT7)
		// DT7遥控器
		DT7_RX_Finish = 1; // 已接受完一包数据
#endif
	}
	// 如果数据来自USART2,即为上位机数据
	/*	if(huart->Instance == USART2 )
		{
				uint8_t Buf[UpperCom_MAX_BUF];
				HAL_UART_Receive_DMA(&huart3,Buf,sizeof(Buf));

				BaseType_t xHigherPriorityTask;
				xQueueSendToBackFromISR(Communicate_ReceivefromPCHandle, Buf, &xHigherPriorityTask);
				portYIELD_FROM_ISR(xHigherPriorityTask);

		}

		//如果数据来自USART1
		if(huart->Instance == USART1)
		{
			memcpy(Fire_RxBuffer,Fire_Rx_Data,sizeof(Fire_Rx_Data));
			Fire_Rx_Finish = 1;//已接收完一包数据
	//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
	//
	//		int temp;
	//		memcpy(&temp, Fire_Rx_Data, 4);
	//		Fric_Data.Required_Speed = temp;
	//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
		}
		*/
}
