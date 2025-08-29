/**
 * @file BSP_Uart.c
 * @author SJW
 * @brief
 * @version 0.1
 * @date 2025-08-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "BSP_Usart.h"

/******************数据定义*****************/
uint8_t vofa_RxData; // 定义接收的数据
float PID_K[3];

/******************接口声明*****************/
Usart_Data_t Usart_Data = Usart_DataGroundInit;
#undef Usart_DataGroundInit
/**
 * @brief  遥控器接收UART中断回调函数
 * @param  void
 * @retval void
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
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

	/************VOFA传参***********/
	else if (huart->Instance == USART1)
	{

		static uint8_t RxIndex;		  // 储存数据数组的索引值
		static uint8_t RxPacket[128]; // 储存数据的数组
		static enum {
			Wait_Head,		   // 等待包头
			Wait_Flag,		   // 等待接收标识
			Wait_Data		   // 等待接收数据
		} RxState = Wait_Head; // 初始状态为等待包头
		static enum {
			CMD_NONE,			 // 空状态
			CMD_Kp,				 // Kp
			CMD_Ki,				 // Ki
			CMD_Kd				 // Kd
		} CurrentCmd = CMD_NONE; // 初始为空状态
		switch (RxState)		 // 使用switch语句使得结构更加清晰
		{
		case Wait_Head:
			if (vofa_RxData == 'K') // 接收到包头‘K’
			{
				RxState = Wait_Flag; // 跳转转态至等待接收标识
			}
			break;
		case Wait_Flag:
			if (vofa_RxData == 'P') // 接收到标识‘P’
			{
				CurrentCmd = CMD_Kp;
				RxState = Wait_Data;
				RxIndex = 0; // 数组清零，便于下一次开始接受实际数据内容
			}
			else if (vofa_RxData == 'I') // 接收到标识‘I’
			{
				CurrentCmd = CMD_Ki;
				RxState = Wait_Data;
				RxIndex = 0;
			}
			else if (vofa_RxData == 'D') // 接收到标识‘D’
			{
				CurrentCmd = CMD_Kd;
				RxState = Wait_Data;
				RxIndex = 0;
			}
			else // 均为接收到，则重置状态
				RxState = Wait_Head;
			break;
		case Wait_Data:
			if (vofa_RxData == 'M') // 接收到包尾‘M’
			{
				RxPacket[RxIndex] = '\0'; // 为接收的数组添加上结束符‘\0’
				uint8_t *endptr;
				float NewValue = strtof((char *)RxPacket, (char **)&endptr); // 双重转换
				// 验证转换有效性
				if (endptr != RxPacket && *endptr == '\0')
				{
					switch (CurrentCmd) // 判断接收标识符
					{
					case CMD_Kp:
						PID_K[0] = NewValue;
						break;
					case CMD_Ki:
						PID_K[1] = NewValue;
						break;
					case CMD_Kd:
						PID_K[2] = NewValue;
						break;
					case CMD_NONE:
						break;
					}
				}
				else
					RxState = Wait_Head;
				CurrentCmd = CMD_NONE; // 状态重置
			}
			else
			{
				if (RxIndex < sizeof(RxPacket) - 1) // 判断数组是否越界
				{
					RxPacket[RxIndex++] = vofa_RxData; // 将接收到的数据存储至数组中
				}
				else
				{
					RxState = Wait_Head; // 溢出复位
				}
			}
			break;
		default:
			RxState = Wait_Head;
			break;
		}
	}
}
