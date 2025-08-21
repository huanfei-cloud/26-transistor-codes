/**
 * @file vofa.h
 * @author SJW
 * @brief
 * @version 2.0
 * @date 2025-8-14
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __VOFA_H
#define __VOFA_H
#include "sys.h"
#include "usart.h"
#include "math.h"

typedef union
{
	float fdata;
	unsigned long ldata;
} FloatLongType;

void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose);

#endif
