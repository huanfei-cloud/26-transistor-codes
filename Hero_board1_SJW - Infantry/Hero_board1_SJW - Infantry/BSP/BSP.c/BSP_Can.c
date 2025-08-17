/**
 * @file BSP_Can.c
 * @author Why (1091537104@qq)
 * @brief Init of CAN
 * @version 0.1
 * @date 2023-08-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "BSP_Can.h"

/*******************************用户数据定义************************************/
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type);
void CAN_SendData(QueueHandle_t CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);

Can_Data_t Can_Data[2] = Can_DataGroundInit;
#undef Can_DataGroundInit
/***********************************接口赋值************************************/
Can_Fun_t Can_Fun = Can_FunGroundInit;
#undef Can_FunGroundInit
/*******************************************************************************/

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t CAN1_0x200_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t CAN1_0x2ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t CAN2_0x1ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t CAN2_0x200_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t CAN2_0x2ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * @Data   2023-08-07
 * @brief  CAN筛选器初始化,其结果是会接收所有ID的
 * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
 * @retval void
 */
static void CAN_FILTER_Init(CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx)
{
  CAN_Filter->FilterFIFOAssignment = CAN_RX_FIFO0; // 筛选器被关联到FIFO0
  CAN_Filter->FilterBank = 0;                      // 筛选器组0
  CAN_Filter->SlaveStartFilterBank = 0;
  CAN_Filter->FilterMode = CAN_FILTERMODE_IDMASK;  // 工作在ID掩码模式
  CAN_Filter->FilterScale = CAN_FILTERSCALE_32BIT; // 筛选器位宽为单个32位。
  CAN_Filter->FilterActivation = ENABLE;           // 使能筛选器
  // 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0//
  CAN_Filter->FilterIdHigh = 0x0000;     // 要筛选的ID高位
  CAN_Filter->FilterIdLow = 0x0000;      // 要筛选的ID低位
  CAN_Filter->FilterMaskIdHigh = 0x0000; // 筛选器高16位每位不须匹配
  CAN_Filter->FilterMaskIdLow = 0x0000;  // 筛选器低16位每位不须匹配
  if (HAL_CAN_ConfigFilter(hcanx, CAN_Filter) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @Data    2021-08-07
 * @brief   canx中断启动
 * @param   CAN_HandleTypeDef *hcanx, uint8_t Can_type
 * @retval  void
 */
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type)
{
  uint8_t Canx_type = Can_type - 1;

  /*使能滤波器*/
  CAN_FILTER_Init(&Can_Data[Canx_type].CAN_FilterTypedef.CAN_Filter, hcanx);
  /*启用CAN*/
  HAL_CAN_Start(hcanx);
  /*使能CAN的IT中断*/
  // HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
  //__HAL_CAN_ENABLE_IT(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
  HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  CAN发送数据
 * @param  CANx	    CAN编号
 * 		id_type 	id类型 CAN_ID_STD， CAN_ID_EXT
 *			id			id号
 * 		data[8]		8个数据
 * @retval None
 */
void CAN_SendData(QueueHandle_t CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8])
{
  Can_Send_Data_t Can_Send_Data;

  Can_Send_Data.Canx = CANx;
  if (id_type == CAN_ID_STD)
  {
    Can_Send_Data.CAN_TxHeader.StdId = id;
  }
  else
  {
    Can_Send_Data.CAN_TxHeader.ExtId = id; // ID号
  }

  Can_Send_Data.CAN_TxHeader.IDE = id_type;      // ID类型
  Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA; // 发送的为数据
  Can_Send_Data.CAN_TxHeader.DLC = 0x08;         // 数据长度为8字节
  Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE;

  memcpy(Can_Send_Data.CANx_Send_RxMessage, data, sizeof(uint8_t[8]));

  xQueueSend(CANx_Handle, &Can_Send_Data, 0);
}

/**
 * @brief  CAN接受数据中断函数
 * @param  CANx 			CAN编号
 * 		    id_type 	id类型 CAN_ID_STD， CAN_ID_EXT
 *			    id				id号
 * 		    data[8]		8个数据
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  static uint8_t Can_Type = 0; // 区别Can1与Can2
  static Can_Export_Data_t Can_Export_Data[2];
  BaseType_t xHigherPriorityTask;
  /*判断是哪一个Can口传回的信息*/
  if (hcan->Instance == CAN1)
    Can_Type = 0;
  else
    Can_Type = 1;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_Data[Can_Type].CAN_RxTypedef.CANx_RxHeader, Can_Data[Can_Type].CAN_RxTypedef.CAN_RxMessage);
  Can_Export_Data[Can_Type].CAN_RxHeader = Can_Data[Can_Type].CAN_RxTypedef.CANx_RxHeader;
  memcpy(&Can_Export_Data[Can_Type].CANx_Export_RxMessage, Can_Data[Can_Type].CAN_RxTypedef.CAN_RxMessage, sizeof(uint8_t[8]));

  /*把接收数据发给接收队列	*/
  if (!Can_Type)
    xQueueSendToBackFromISR(CAN1_ReceiveHandle, &Can_Export_Data[Can_Type], &xHigherPriorityTask);
  else
    xQueueSendToBackFromISR(CAN2_ReceiveHandle, &Can_Export_Data[Can_Type], &xHigherPriorityTask);
  portYIELD_FROM_ISR(xHigherPriorityTask);
}
