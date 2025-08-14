/**
 * @file task_init.c
 * @author xyz
 * @brief 完成相关任务的板级支持包启动
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#include "task_init.h"
#include "chassis.h"
#include "bsp_usart.h"
#include "Protocol_Judgement.h"
#include "DT7.h"
#include "bsp_fdcan.h"
#include "IMU.h"

void task_start_init(void)
{
	
	bsp_can_init(); 
	chassis_init(&chassis_control);
	DT7_Init();
	IMU_Init();
	JudgeSystem_USART_Receive_DMA(&huart3);  //开启裁判系统

}
