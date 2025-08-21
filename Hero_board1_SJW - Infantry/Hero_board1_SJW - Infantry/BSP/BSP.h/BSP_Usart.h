#ifndef __BSP_USART
#define __BSP_USART


#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "SBUS.h"
#include "DT7.h"
#include "Extern_Handles.h"
#include "FrictionWheel.h"

#define Usart_DMA_Idle_Length 128

#define Usart_DataGroundInit \
    {                        \
        {0},                 \
    }

typedef struct
{
    uint8_t Usart6_Data[Usart_DMA_Idle_Length];
    uint16_t Data_Size;
} Usart_Data_t;

extern Usart_Data_t Usart_Data;

#endif
