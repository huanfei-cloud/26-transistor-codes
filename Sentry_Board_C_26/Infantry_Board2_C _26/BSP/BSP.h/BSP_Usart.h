#ifndef __BSP_USART
#define __BSP_USART

#include "M2006_Motor.h"
#include "M6020_Motor.h"

#include <stdio.h>
#include <string.h>
#include "usart.h"

#define Usart_DMA_Idle_Length 128

#define Usart_FunGroundInit           \
    {                              		\
        &Usart_DMA_Receive_Init,      \
    }

#define Usart_DataGroundInit \
    {                      \
        {0},         			 \
    }

typedef struct
{
	uint8_t Usart6_Data[Usart_DMA_Idle_Length];
	uint16_t Data_Size;
} Usart_Data_t;

typedef struct
{
//    void (*Send_IMU_Data)(void);
    void (*Usart_DMA_Receive_Init)();
} Usart_Fun_t;

extern Usart_Fun_t Usart_Fun;
extern Usart_Data_t Usart_Data;

#endif
