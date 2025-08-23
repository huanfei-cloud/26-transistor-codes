 /**
 * @file Task_OffLineCheck.c
 * @author Cyx
 * @brief 单独调试某个电机时可不启用此任务
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_OffLineCheck.h"
#include "Shoot.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"
#include "SBUS.h"
#include "DT7.h"

#include "BSP_Fdcan.h"
#include "BSP_Test.h"
static uint16_t pitch_Frame = 0;
static uint16_t fricL_Frame = 0;
static uint16_t fricR_Frame = 0;
static uint16_t dial_Frame = 0;

void Off_Line_Check(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(100); //每100毫秒强制进入数据发送
    for (;;)
    {
			//Pitch掉线检测
				if(pitch_Frame == M6020s_Pitch.InfoUpdateFrame)
				{
					M6020s_Pitch.InfoUpdateFlag = 0;
					M6020s_Pitch.InfoUpdateFrame = 0;
				}
				pitch_Frame = M6020s_Pitch.InfoUpdateFrame ;
				
			//FricL掉线检测
				if(fricL_Frame == M3508_Array[FricL_Wheel].InfoUpdateFrame)
				{
					M3508_Array[FricL_Wheel].InfoUpdateFlag = 0;
					M3508_Array[FricL_Wheel].InfoUpdateFrame = 0 ;
					ControlMes.fric_Flag = 0;
				}
				fricL_Frame = M3508_Array[FricL_Wheel].InfoUpdateFrame ;
				
			//FricR掉线检测
				if(fricR_Frame == M3508_Array[FricR_Wheel].InfoUpdateFrame)
				{
					M3508_Array[FricR_Wheel].InfoUpdateFlag = 0;
					M3508_Array[FricR_Wheel].InfoUpdateFrame = 0 ;				
					ControlMes.fric_Flag = 0;
				}
				fricR_Frame = M3508_Array[FricR_Wheel].InfoUpdateFrame ;
				
			//Dial掉线检测
				if(dial_Frame == M2006_Array[Dial_Motor].InfoUpdateFrame )
				{
					M2006_Array[Dial_Motor].InfoUpdateFlag = 0;
					M2006_Array[Dial_Motor].InfoUpdateFrame = 0;
					M2006_Array[Dial_Motor].targetSpeed = 0;
				}
				dial_Frame = M2006_Array[Dial_Motor].InfoUpdateFrame ;
				
		
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
