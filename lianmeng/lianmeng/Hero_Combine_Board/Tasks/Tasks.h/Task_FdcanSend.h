/**
 * @file Task_FdcanSend.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __TASK_FDCANSEND_H
#define __TASK_FDCANSEND_H

#include "typedef.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Extern_Handles.h"

void AllFdcanSend(void const *argument);


#endif
