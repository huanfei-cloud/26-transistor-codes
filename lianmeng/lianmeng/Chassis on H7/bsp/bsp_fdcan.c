/**
 * @file bsp_fdcan.c
 * @author lyd
 * @brief CAN通信初始化与配置流程
 * @version 1.1
 * @date 2024-09-13
 * @copyright Transistor BUAA
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_fdcan.h"
#include "usart.h"
#include "motor.h"
#include "chassis.h"
#include "BoardCommunication.h"
#include "PowerBank.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
// 控制yaw轴6020电机       205-208
uint8_t CAN_0x1ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 控制底盘四个3508电机    201-204
uint8_t CAN_0x200_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 控制云台俯仰 偏航6020电机
uint8_t CAN_0x2ff_Tx_Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

void bsp_can_init(void)
{

	FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN1_FilterConfig.FilterIndex = 0;
	FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN1_FilterConfig.FilterID1 = 0x00000000;
	FDCAN1_FilterConfig.FilterID2 = 0x00000000;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN2_FilterConfig.FilterIndex = 0;
	FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
	FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN2_FilterConfig.FilterID1 = 0x00000000;
	FDCAN2_FilterConfig.FilterID2 = 0x00000000;

	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
 * @param:       hfdcan：FDCAN句柄
 * @param:       id：CAN设备ID
 * @param:       data：发送的数据
 * @param:       len：发送的数据长度
 * @retval:     	void
 * @details:    	发送数据
 **/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef pTxHeader;
	pTxHeader.Identifier = id;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;

	if (len <= 8)
		pTxHeader.DataLength = len;
	if (len == 12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if (len == 16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if (len == 20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if (len == 24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if (len == 32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if (len == 48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if (len == 64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) == HAL_OK)
		return 1; // 发送
	else
		return 0;
}

/**
 * @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
 * @param:       hfdcan：FDCAN句柄
 * @param:       buf：接收数据缓存
 * @retval:     	接收的数据长度
 * @details:    	接收数据
 **/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_12)
			len = 12;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_16)
			len = 16;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_20)
			len = 20;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_24)
			len = 24;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_32)
			len = 32;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_48)
			len = 48;
		if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_64)
			len = 64;

		return len; // 接收数据
	}
	return 0;
}

void fdcan1_rx_callback(FDCAN_HandleTypeDef *hfdcan)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(hfdcan, &rec_id, rx_data); // 调用中断的id传进rec_id中
	switch (rec_id)
	{
	case 0x201:
	{
		get_motor_data(&chassis_control.motor1,rx_data);
		motor_encoder_state_change(&chassis_control.motor1);
		break;
	}
	
	case 0x202:
	{
		get_motor_data(&chassis_control.motor2,rx_data);
		motor_encoder_state_change(&chassis_control.motor2);
		break;
	}
	
	case 0x203:
	{
		get_motor_data(&chassis_control.motor3,rx_data);
		motor_encoder_state_change(&chassis_control.motor3);
		break;
	}
	
	case 0x204:
	{
		get_motor_data(&chassis_control.motor4,rx_data);
		motor_encoder_state_change(&chassis_control.motor4);
		break;
	}
	
	
	case 0x209:
	{
//		get_motor_data(&yaw_motor,rx_data);
//		motor_encoder_state_change(&yaw_motor);
//		yaw_control.UpdateFrame++;
//		yaw_control.UpdateFlag = 1;
		break;
	}
	
	case 0x51:
	{
		PowerBankReceive(rx_data);
		break;
	}
	
	}
}	

void fdcan2_rx_callback(FDCAN_HandleTypeDef *hfdcan)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(hfdcan, &rec_id, rx_data); // 调用中断的id传进rec_id中
	switch (rec_id)
	{
	case 0x205:
	{
		get_motor_data(&chassis_control.joint_motor1,rx_data);
		motor_encoder_state_change(&chassis_control.joint_motor1);
		break;
	}
	
	case 0x206:
	{
		get_motor_data(&chassis_control.joint_motor2,rx_data);
		motor_encoder_state_change(&chassis_control.joint_motor2);
		break;
	}
	
	case 0x207:
	{
		get_motor_data(&chassis_control.joint_motor3,rx_data);
		motor_encoder_state_change(&chassis_control.joint_motor3);
		break;
	}
	
	case 0x208:
	{
		get_motor_data(&chassis_control.joint_motor4,rx_data);
		motor_encoder_state_change(&chassis_control.joint_motor4);
		break;
	}
	
	case CAN_ID_CHASSIS:
	{
		Board2_FUN.Board2_getChassisInfo(rx_data);
		break;
	}
	
	case CAN_ID_GIMBAL:
	{
		Board2_FUN.Board2_getGimbalInfo(rx_data);
		break;
	}
	
	
	case CAN_ID_CHASSIS_FUNCTION:
	{
		Board2_FUN.Board2_getChassisFunctionInfo(rx_data);
		break;
	}
	
	}
}	

/**
 * @brief HAL库FDCAN接收FIFO0中断
 *
 * @param hfdcan FDCAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
// 此函数是CAN接收FIFO0消息挂起回调函数，用于处理CAN1和CAN2的接收消息。
{
	// 选择回调函数
	if (hfdcan->Instance == FDCAN1)
	{
		fdcan1_rx_callback(&hfdcan1);
	}
	if (hfdcan->Instance == FDCAN2)
	{
		fdcan2_rx_callback(&hfdcan2);
	}
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hfdcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
{
	// 选择回调函数
	if (hfdcan->Instance == FDCAN1)
	{
		fdcan1_rx_callback(&hfdcan1);
	}
	else if (hfdcan->Instance == FDCAN2)
	{
		fdcan2_rx_callback(&hfdcan2);
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (ErrorStatusITs & FDCAN_IR_BO)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
	if (ErrorStatusITs & FDCAN_IR_EP)
	{
		MX_FDCAN1_Init();
		bsp_can_init();
	}
}
