/**
 * @file Task_CanSend.c
 * @author Miraggio (w1159904119@gmail)
 * @brief
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_CanSend.h"

/**
 * @brief Can队列发送
 *
 */
void AllCanSend(void const *argument)
{
    uint32_t CAN_Send_MailBox;
    Can_Send_Data_t Can_Send_Data;
    for (;;)
    {
        xQueueReceive(CAN_SendHandle, &Can_Send_Data, portMAX_DELAY);
        HAL_CAN_AddTxMessage(Can_Send_Data.Canx, &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage, &CAN_Send_MailBox);
    }
}
