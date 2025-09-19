/**
 * @file Task_OffLineCheck.c
 * @author Cyx��SJW
 * @brief ���������߼��
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_OffLineCheck.h"

static uint16_t fricF_1_Frame = 0;
static uint16_t fricB_1_Frame = 0;
static uint16_t fricF_2_Frame = 0;
static uint16_t fricB_2_Frame = 0;
static uint16_t fricF_3_Frame = 0;
static uint16_t fricB_3_Frame = 0;
static uint16_t dial_Frame = 0;

void Off_Line_Check(void const *argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(100);
	for (;;)
	{

		if (fricF_1_Frame == M3508_Array[Fric_Front_1].InfoUpdateFrame)
		{
			M3508_Array[Fric_Front_1].InfoUpdateFlag = 0;
			M3508_Array[Fric_Front_1].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricF_1_Frame = M3508_Array[Fric_Front_1].InfoUpdateFrame;

		if (fricF_2_Frame == M3508_Array[Fric_Front_1].InfoUpdateFrame)
		{
			M3508_Array[Fric_Front_2].InfoUpdateFlag = 0;
			M3508_Array[Fric_Front_2].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricF_2_Frame = M3508_Array[Fric_Front_2].InfoUpdateFrame;

		if (fricF_3_Frame == M3508_Array[Fric_Front_3].InfoUpdateFrame)
		{
			M3508_Array[Fric_Front_3].InfoUpdateFlag = 0;
			M3508_Array[Fric_Front_3].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricF_3_Frame = M3508_Array[Fric_Front_3].InfoUpdateFrame;

		if (fricB_1_Frame == M3508_Array[Fric_Back_1].InfoUpdateFrame)
		{
			M3508_Array[Fric_Back_1].InfoUpdateFlag = 0;
			M3508_Array[Fric_Back_1].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricB_2_Frame = M3508_Array[Fric_Back_1].InfoUpdateFrame;

		if (fricB_2_Frame == M3508_Array[Fric_Back_2].InfoUpdateFrame)
		{
			M3508_Array[Fric_Back_2].InfoUpdateFlag = 0;
			M3508_Array[Fric_Back_2].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricB_2_Frame = M3508_Array[Fric_Back_2].InfoUpdateFrame;

		if (fricB_3_Frame == M3508_Array[Fric_Back_3].InfoUpdateFrame)
		{
			M3508_Array[Fric_Back_3].InfoUpdateFlag = 0;
			M3508_Array[Fric_Back_3].InfoUpdateFrame = 0;
			ControlMes.fric_Flag = 0;
		}
		fricB_3_Frame = M3508_Array[Fric_Back_3].InfoUpdateFrame;

		if (dial_Frame == M3508_Array[Dial_Wheel].InfoUpdateFrame)
		{
			M3508_Array[Dial_Wheel].InfoUpdateFlag = 0;
			M3508_Array[Dial_Wheel].InfoUpdateFrame = 0;
			M3508_Array[Dial_Wheel].targetSpeed = 0;
		}
		dial_Frame = M3508_Array[Dial_Wheel].InfoUpdateFrame;

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}
