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

enum
{
    M2006,          /* 2006Ω” ’ */
};

void Can1Receives(void const *argument);
void Can2Receives(void const *argument);

#endif /*__TASK_CANMSG_H*/
