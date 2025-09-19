 /**
 * @file Task_RobotControl.c
 * @author rm_control_team
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_RobotControl.h"
#include "Shoot.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"
#include "SBUS.h"
#include "DT7.h"

#include "BSP_Fdcan.h"
#include "BSP_Test.h"
void Robot_Control(void const *argument)
{
	  
    portTickType xLastWakeTime;//记录任务上次唤醒的时间点，用于精确周期控制。
    xLastWakeTime = xTaskGetTickCount();//获取当前系统时钟计数。
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每3毫秒强制进入数据发送
    for (;;)
    {
			
			Cloud_FUN.Remote_Change();     //变速小陀螺
			Cloud_FUN.Cloud_Sport_Out();   
			Shoot_Fun.Shoot_Processing();

/****************发送电流数据***************/
    uint8_t data1[8],data2[8];
			
		M3508_FUN.M3508_setCurrent(M3508_Array[FricL_Wheel].outCurrent, M3508_Array[FricR_Wheel].outCurrent, M2006_Array[Dial_Motor].outCurrent,M2006_Array[Dial_Motor].outCurrent,data1);

		J4310_Fun.J4310_setParameter(J4310s_Pitch.outPosition,J4310s_Pitch.outSpeed,J4310s_Pitch.outKp,J4310s_Pitch.outKd,J4310s_Pitch.outTorque,data2);

		Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan1, FDCAN_STANDARD_ID, M3508_SENDID_Fric_Dial, data1); 
			
		Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, J4310_SENDID_Pitch, data2); 
/****************发送电流数据 end***************/                
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
