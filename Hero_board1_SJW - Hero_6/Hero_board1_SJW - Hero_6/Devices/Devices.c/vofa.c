/**
 * @file vofa.c
 * @author SJW
 * @brief
 * @version 2.0
 * @date 2025-8-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "vofa.h"

/***************函数声明**************/
void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose);
void VOFA_Init(void);

/***************数据声明**************/
const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
uint8_t c_data[4];

/**
 * @Data    2025-08-28
 * @brief   将32位浮点数转换为4个字节的数组
 * @param   float f: 需要转换的浮点数
 * @param   uint8_t *c: 存储转换结果的字节数组
 * @retval  void
 */
void float_turn_u8(float f, uint8_t *c)
{
	uint8_t x;
	FloatLongType data;
	data.fdata = f;

	for (x = 0; x < 4; x++)
	{
		c[x] = (uint8_t)(data.ldata >> (x * 8));
	}
}

/**
 * @Data    2025-08-28
 * @brief   通过串口发送一个浮点数数组，并附加结束标记
 * @param   float *fdata: 需要转换的浮点数数组
 * @param   uint16_t fdata_num: 浮点数数组的长度
 * @param   USART_TypeDef *Usart_choose: 选择的USART
 * @retval  void
 */
void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose)
{
	uint16_t x;
	uint8_t y;
	// 发送所有浮点数据
	for (x = 0; x < fdata_num; x++)
	{
		// 将当前浮点数转换为字节数组
		float_turn_u8(fdata[x], c_data);

		// 通过串口发送4个字节
		for (y = 0; y < 4; y++)
		{
			Usart_choose->DR = c_data[y];		   // 写入数据寄存器
			while ((Usart_choose->SR & 0X40) == 0) // 等待发送完成
				;
		}
	}

	// 发送结束标记
	for (y = 0; y < 4; y++)
	{
		Usart_choose->DR = tail[y];
		while ((Usart_choose->SR & 0X40) == 0)
			;
	}
}

/**
 * @brief  VOFA接收初始化
 * @param  void
 * @retval void
 */
void VOFA_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &vofa_RxData, 1);
}
