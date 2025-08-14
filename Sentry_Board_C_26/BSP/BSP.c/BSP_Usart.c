#include "BSP_Usart.h"
#include "Omni_Chassis.h"
#include "BSP_Test.h"
#include "Extern_Handles.h"
#include "Saber_C3.h"
#include "N100.h"
#include "DT7.h"
#include "Protocol_Judgement.h"

/***************用户数据声明****************/
void Send_IMU_Data(void);
void Usart_DMA_Receive_Init(void);
//extern fp32 INS_angle[3];
/******************接口声明*****************/
Usart_Fun_t Usart_Fun = Usart_FunGroundInit;
#undef Usart_FunGroundInit
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit



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
	static uint8_t Saber_Montage_Flag = 0;            //表示陀螺仪数据的拼接起点
	
	//如果数据来自USART1,即为IMU数据
	if(huart->Instance == USART1)
	{
		#if(Imu_Device == Saber)
		if(Saber_Montage_Flag)
		{
			memcpy(Saber_Montage + Saber_Data_Length, Saber_Rxbuffer, Saber_Data_Length);
			Saber_Montage_Flag = 0;
		}
		else
		{
			memcpy(Saber_Montage, Saber_Rxbuffer, Saber_Data_Length);
			Saber_Montage_Flag = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Saber_Rxbuffer,sizeof(Saber_Rxbuffer));
		#elif(Imu_Device == N100)
		if(N100_Rxbuffer[0] == FRAME_HEAD && N100_Rxbuffer[55] == FRAME_END)
		{
			N100_Read();
		}
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart1,N100_Rxbuffer,sizeof(N100_Rxbuffer));
		#endif
//		memcpy(Fire_RxBuffer,Fire_Rx_Data,sizeof(Fire_Rx_Data));
//		Fire_Rx_Finish = 1;//已接收完一包数据
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
//		
//		int temp;
//		memcpy(&temp, Fire_Rx_Data, 4);
//		Fric_Data.Required_Speed = temp;
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Fire_Rx_Data,FIRE_MAX_BUF_LEN);
	}
    if(huart->Instance == USART6)
    {
        JudgeSystem_Handler(&huart6);
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Saber_Read();
		HAL_UART_Receive_DMA(&huart1,Saber_Rxbuffer,sizeof(Saber_Rxbuffer)); 
	}
}
	

