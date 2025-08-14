/**
 * @file Task_Online_Check.c
 * @author Lxr
 * @brief 
 * @version 0.1
 * @date 2024-01-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
 
 #include "Task_Online_Check.h"
 #include "DT7.h"
 
 /**
 * @brief 检测任务
 * 
 * @param argument 
 * @return  
 */
WorldTime_RxTypedef Online_Check_FPS;
uint32_t OnChTest_FPS;
void Online_Check(void const *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(200);//每200毫秒强制进入总控制
	
//	DevicesMonitor_FUN.DevicesMonitor_Init();
	
	for (;;)
	{
		
		DevicesMonitor_FUN.DevicesMonitor_update();
		
//		DevicesMonitor_FUN.DevicesMonitor_Alert();
//	if(Robot_FPS == 500 && GetData_FPS == 1000 && DT7_Fun.DR16_DataCheck == 0)
//		{
//			HAL_IWDG_Refresh(&hiwdg); //喂狗
//		}
		Get_FPS(&Online_Check_FPS, &OnChTest_FPS);	
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement );
	}
}
