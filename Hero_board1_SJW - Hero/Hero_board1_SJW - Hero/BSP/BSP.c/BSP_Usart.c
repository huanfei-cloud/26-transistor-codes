#include "BSP_Usart.h"


/******************接口声明*****************/
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit
/**
 * @brief  ��ʾ��ǰ�ٶ�����
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

}
