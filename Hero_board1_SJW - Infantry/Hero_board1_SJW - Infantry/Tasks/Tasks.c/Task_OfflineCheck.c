/**
 * @file Task_OffLineCheck.c
 * @author Cyx
 * @brief
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_OffLineCheck.h"

static uint16_t fricL_Frame = 0;
static uint16_t fricR_Frame = 0;
static uint16_t dial_Frame = 0;

void Off_Line_Check(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(100);
	for (;;)
	{
		if (fricL_Frame == M3508_Array[FricL_Wheel].InfoUpdateFrame)
		{
			M3508_Array[FricL_Wheel].InfoUpdateFlag = 0;
			M3508_Array[FricL_Wheel].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricL_Frame = M3508_Array[FricL_Wheel].InfoUpdateFrame;

		if (fricR_Frame == M3508_Array[FricR_Wheel].InfoUpdateFrame)
		{
			M3508_Array[FricR_Wheel].InfoUpdateFlag = 0;
			M3508_Array[FricR_Wheel].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricR_Frame = M3508_Array[FricR_Wheel].InfoUpdateFrame;

		if (dial_Frame == M2006_Array[Dial_Motor].InfoUpdateFrame)
		{
			M2006_Array[Dial_Motor].InfoUpdateFlag = 0;
			M2006_Array[Dial_Motor].InfoUpdateFrame = 0;
			M2006_Array[Dial_Motor].targetSpeed = 0;
		}
		dial_Frame = M2006_Array[Dial_Motor].InfoUpdateFrame;

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
