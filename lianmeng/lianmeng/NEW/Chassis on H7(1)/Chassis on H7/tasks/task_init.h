/**
 * @file task_init.h
 * @author xyz
 * @brief 完成相关任务的板级支持包启动
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#ifndef TASK_INIT_H
#define TASK_INIT_H
#include "fdcan.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "bsp_usart.h"
#include "pid.h"
#include "motor.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

void task_start_init(void);

#endif
