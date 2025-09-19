/**
 * @file Task_CanSend.c
 * @author Miraggio (w1159904119@gmail),zs
 * @brief
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_FdcanSend.h"
#include "BSP_FDCAN.h"

/**
 * @brief FDCan∂”¡–∑¢ÀÕ
 *
 */
void AllFdcanSend(void const *argument)
{
    Fdcan_Send_Data_t Fdcan_Send_Data;
    for (;;)
    {
        xQueueReceive(FDCAN_SendHandle, &Fdcan_Send_Data, portMAX_DELAY);
				HAL_FDCAN_AddMessageToTxFifoQ(Fdcan_Send_Data.Fdcanx,&Fdcan_Send_Data.FDCAN_TxHeader,Fdcan_Send_Data.FDCANx_Send_RxMessage);

		}
}
