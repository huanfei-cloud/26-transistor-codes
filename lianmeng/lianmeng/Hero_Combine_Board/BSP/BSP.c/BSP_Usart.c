#include "BSP_Usart.h"
#include "Omni_Chassis.h"
#include "SBUS.h"
#include "DT7.h"
#include "BSP_Test.h"
#include "Extern_Handles.h"
#include "FrictionWheel.h"
#include "Saber_C3.h"
#include "Protocol_Judgement.h"

/***************�û���������****************/
void Send_Motor_Data(M2006s_t *M2006_Array);
void Send_IMU_Data(void);
/******************�ӿ�����*****************/
//Usart_Fun_t Usart_Fun = Usart_FunGroundInit;
//#undef Usart_FunGroundInit
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit


/**
  * @brief  ���տ��лص�
  * @param  void
  * @retval void
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint8_t Saber_Montage_Flag = 0;            //��ʾ���������ݵ�ƴ�����
//	BaseType_t xHigherPriorityTask;        //FreeROTS�˳��ж�ʱ�ж��Ƿ�Ҫ���������л�
	
    //�����������USART2,��Ϊң��������
	if(huart->Instance == USART2 )
    {
		//DT7ң����
			DT7_RX_Finish = 1;//�ѽ�����һ������
    }
    //�����������USART3,��Ϊ��λ������
	if(huart->Instance == USART3 )
    {
			uint8_t Buf[UpperCom_MAX_BUF];
			HAL_UART_Receive_DMA(&huart3,Buf,sizeof(Buf));		
			
			BaseType_t xHigherPriorityTask;
			xQueueSendToBackFromISR(Communicate_ReceivefromPCHandle, Buf, &xHigherPriorityTask);
			portYIELD_FROM_ISR(xHigherPriorityTask);

    }
	
	//�����������USART1,��ΪIMU����
	if(huart->Instance == USART1)
	{
#if(USING_BOARD == BOARD2)
//		if(Saber_Montage_Flag)
//		{
//			memcpy(Saber_Montage + Saber_Data_Length, Saber_Rxbuffer, Saber_Data_Length);
//			Saber_Montage_Flag = 0;
//		}
//		else
//		{
//			memcpy(Saber_Montage, Saber_Rxbuffer, Saber_Data_Length);
//			Saber_Montage_Flag = 1;
//		}
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Saber_Rxbuffer,sizeof(Saber_Rxbuffer));
#endif
		JudgeSystem_FUN.JudgeSystem_Handler(&huart1);
	//JudgeSystem_FUN.JudgeSystem_USART_Receive_DMA(&huart1);
	}
	
//	if(huart->Instance == USART6)
//	{
//		Usart_Data.Data_Size = Size;
//		xQueueSendToBackFromISR(Judge_ReceiveHandle, &Usart_Data,
//                                &xHigherPriorityTask);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Usart_Data.Usart6_Data,
//									 Usart_DMA_Idle_Length);
//		portYIELD_FROM_ISR(xHigherPriorityTask);
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Saber_Fun.Saber_Read();
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Saber_Rxbuffer,sizeof(Saber_Rxbuffer)); 
	}
}
	
