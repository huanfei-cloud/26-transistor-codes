/**
 * @file Task_CanMsg.h
 * @author Why (1091537104@qq)
 * @brief
 * @version 0.1
 * @date 2023-08-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H

#include "cmsis_os.h"
#include "queue.h"
#include "Extern_Handles.h"
#include "BSP_CAN.h"
#include "PowerControl.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Cloud_Control.h"
#include "BSP_BoardCommunication.h"
#include "BSP_Usart.h"
#include "J4310_Motor.h"

extern uint16_t pitch_online_flag;

enum
{
    M2006, /* 2006µç»ú */
};

void Can1Receives(void const *argument);
void Can2Receives(void const *argument);

#endif /*__TASK_CANMSG_H*/
