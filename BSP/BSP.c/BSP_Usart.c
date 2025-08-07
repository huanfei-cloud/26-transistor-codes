#include "BSP_Usart.h"
#include "SBUS.h"
#include "DT7.h"
#include "BSP_Test.h"
#include "Extern_Handles.h"
#include "FrictionWheel.h"
#include "Saber_C3.h"

/***************用户数据声明****************/
void Send_Motor_Data(M2006s_t *M2006_Array);
void Send_IMU_Data(void);
void Usart_DMA_Receive_Init(void);
//extern fp32 INS_angle[3];
/******************接口声明*****************/
Usart_Fun_t Usart_Fun = Usart_FunGroundInit;
#undef Usart_FunGroundInit
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit

/**
  * @brief  显示当前速度数据
  * @param  M2006s_t结构体指针
  * @retval void
  */
void Send_Motor_Data(M2006s_t *M2006_Array)
{
    uint8_t message[30];

    sprintf((char *)message, "v1=%d v2=%d v3=%d v4=%d\n\r",
            M2006_Array[0].realSpeed,
            M2006_Array[1].realSpeed,
            M2006_Array[2].realSpeed,
            M2006_Array[3].realSpeed);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)message,
                          strlen((const char *)message));
    //Usart1_Printf("v1=%d v2=%d v3=%d v4=%d\r\n",v1,v2,v3,v4);
}

/**
  * @brief  串口DMA接收初始化
  * @param  void
  * @retval void
  */
void Usart_DMA_Receive_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Usart_Data.Usart6_Data,
                                 Usart_DMA_Idle_Length);

}

/**
  * @brief  接收空闲回调
  * @param  void
  * @retval void
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// BaseType_t xHigherPriorityTask;        //FreeROTS退出中断时判断是否要进行任务切换
    //如果数据来自USART2,即为遥控器数据
	if(huart->Instance == USART2 )
    {
#if (RemoteControlMethod == TDF)
		{//天地飞遥控器
			memcpy(SBUS_RXBuffer,SBUS_Rx_Data,sizeof(SBUS_Rx_Data));
			SBUS_RX_Finish=1;//已接收完一包数据
		}
#elif (RemoteControlMethod == DT7)
		//DT7遥控器
			DT7_RX_Finish = 1;//已接受完一包数据
#endif
		
    }
    //如果数据来自USART3,即为上位机数据
	if(huart->Instance == USART3 )
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
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
//		
//		int temp;
//		memcpy(&temp, Fire_Rx_Data, 4);
//		Fric_Data.Required_Speed = temp;
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
	}
	
}


	

