/**
 * @file Task_Sampling.h
 * @author Lxr 
 * @brief 
 * @version 0.1
 * @date 2024-01-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
 #include "Task_Sampling.h"
 
 #include "DT7.h"
 #include "Saber_C3.h"

WorldTime_RxTypedef Sampling_FPS;
uint32_t GetData_FPS;

void Fixed_Sampling(void const *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    const TickType_t TimeIncrement = pdMS_TO_TICKS(3); //每一毫秒强制进入总控制
	
#if(USING_BOARD == BOARD1)
	for (;;)
	{
		//Saber_Fun.Saber_Read();
		DT7_Fun.DT7_Handle();//接收遥控器数据
		
		Get_FPS(&Sampling_FPS, &GetData_FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
#elif(USING_BOARD == BOARD2)
	for (;;)
	{
		Saber_Fun.Saber_Read();//接收IMU数据
		Board2_FUN.Board2_To_1();
		Get_FPS(&Sampling_FPS, &GetData_FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
#endif
}
