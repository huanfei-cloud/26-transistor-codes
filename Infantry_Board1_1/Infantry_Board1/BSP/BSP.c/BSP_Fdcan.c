/**
 * @file BSP_Can.c
 * @author ZS (2729511164@qq)
 * @brief Init of FDCAN
 * @version 0.1
 * @date 2024-11-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Extern_Handles.h"
#include "fdcan.h"
#include "BSP_Fdcan.h"

/*******************************�û����ݶ���************************************/
void FDCAN_IT_Init(FDCAN_HandleTypeDef *hfdcanx, uint8_t Fdcan_type);
void FDCAN_SendData(QueueHandle_t FDCANx_Handle, FDCAN_HandleTypeDef *FDCANx, uint8_t id_type, uint32_t id, uint8_t data[8]);

Fdcan_Data_t Fdcan_Data[2] = Fdcan_DataGroundInit;
#undef Fdcan_DataGroundInit
/***********************************�ӿڸ�ֵ************************************/
Fdcan_Fun_t Fdcan_Fun = Fdcan_FunGroundInit;
#undef Fdcan_FunGroundInit
/*******************************************************************************/

/**
  * @Data   2023-08-07
  * @brief  FDCANɸѡ����ʼ��,�����ǻ��������ID��
  * @param  FDCAN_FilterTypeDef *FDCAN_Filter, FDCAN_HandleTypeDef *hfdcanx
  * @retval void
  */
static void FDCAN_FILTER_Init(FDCAN_FilterTypeDef *FDCAN_Filter, FDCAN_HandleTypeDef *hfdcanx)
{
	FDCAN_Filter->IdType = FDCAN_STANDARD_ID;
  FDCAN_Filter->FilterIndex = 0;
  FDCAN_Filter->FilterType = FDCAN_FILTER_MASK;
  FDCAN_Filter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN_Filter->FilterID1 = 0x00000000; 
  FDCAN_Filter->FilterID2 = 0x00000000; 
	if(HAL_FDCAN_ConfigFilter(hfdcanx, FDCAN_Filter) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_FDCAN_ConfigGlobalFilter(hfdcanx, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @Data    2021-08-07
  * @brief   fdcanx�ж�����
  * @param   FDCAN_HandleTypeDef *fdhcanx, uint8_t fdcan_type
  * @retval  void
  */
void FDCAN_IT_Init(FDCAN_HandleTypeDef *hfdcanx, uint8_t Fdcan_type)
{
    uint8_t Fdcanx_type = Fdcan_type - 1;
	
		/*ʹ���˲���*/
    FDCAN_FILTER_Init(&Fdcan_Data[Fdcanx_type].FDCAN_FilterTypedef.FDCAN_Filter, hfdcanx);

		/*����FDCAN*/
		if (HAL_FDCAN_Start(hfdcanx) != HAL_OK)
		{
			Error_Handler();
		}
    /*ʹ��FDCAN��IT�ж�*/
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
			Error_Handler();
    }
		

}

/**
  * @brief  FDCAN��������
  * @param  FDCANx	    FDCAN���
  * 		id_type 	id���� FDCAN_ID_STD�� FDCAN_ID_EXT
  *			id			id��
  * 		data[8]		8������
  * @retval None
  */
void FDCAN_SendData(QueueHandle_t	FDCANx_Handle, FDCAN_HandleTypeDef *FDCANx, uint8_t id_type, uint32_t id, uint8_t data[8])
{
    Fdcan_Send_Data_t Fdcan_Send_Data;

    Fdcan_Send_Data.Fdcanx = FDCANx;

    Fdcan_Send_Data.FDCAN_TxHeader.Identifier = id;
    Fdcan_Send_Data.FDCAN_TxHeader.IdType = id_type;      //ID����
    Fdcan_Send_Data.FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME; //���͵�Ϊ����
    Fdcan_Send_Data.FDCAN_TxHeader.DataLength = 8;         //���ݳ���Ϊ8�ֽ�
    Fdcan_Send_Data.FDCAN_TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;  //���ͽڵ㴦����������״̬
		Fdcan_Send_Data.FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        //��ת��������
		Fdcan_Send_Data.FDCAN_TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;        //����CANģʽ
		Fdcan_Send_Data.FDCAN_TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
		Fdcan_Send_Data.FDCAN_TxHeader.MessageMarker = 0;

    memcpy(Fdcan_Send_Data.FDCANx_Send_RxMessage,
           data,
           sizeof(uint8_t[8]));

    xQueueSend(FDCANx_Handle, &Fdcan_Send_Data, 0);
}


/**
  * @brief  FDCAN���������жϺ���
  * @param  FDCANx 			FDCAN���
  * 		    id_type 	id���� FDCAN_ID_STD�� FDCAN_ID_EXT
  *			    id				id��
  * 		    data[8]		8������
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static uint8_t Fdcan_Type = 0;                     //����Fdcan1��Fdcan2
    static Fdcan_Export_Data_t Fdcan_Export_Data[2];
    BaseType_t xHigherPriorityTask;
    /*�ж�����һ��Fdcan�ڴ��ص���Ϣ*/
    if(hfdcan->Instance == FDCAN1) Fdcan_Type = 0;
    else Fdcan_Type = 1;

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                         &Fdcan_Data[Fdcan_Type].FDCAN_RxTypedef.FDCANx_RxHeader,
                         Fdcan_Data[Fdcan_Type].FDCAN_RxTypedef.FDCAN_RxMessage);

    Fdcan_Export_Data[Fdcan_Type].FDCAN_RxHeader = Fdcan_Data[Fdcan_Type].FDCAN_RxTypedef.FDCANx_RxHeader;
    memcpy(&Fdcan_Export_Data[Fdcan_Type].FDCANx_Export_RxMessage,
           Fdcan_Data[Fdcan_Type].FDCAN_RxTypedef.FDCAN_RxMessage,
           sizeof(uint8_t[8]));

    /*�ѽ������ݷ������ն���	*/
    if(!Fdcan_Type) xQueueSendToBackFromISR(FDCAN1_ReceiveHandle,
                                              &Fdcan_Export_Data[Fdcan_Type],
                                              &xHigherPriorityTask);
    else xQueueSendToBackFromISR(FDCAN2_ReceiveHandle,
                                     &Fdcan_Export_Data[Fdcan_Type],
                                     &xHigherPriorityTask);
    portYIELD_FROM_ISR(xHigherPriorityTask);
}

