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
#include "DT7.h"
#include "Protocol_UpperComputer.h"
#include "Task_Online_Check.h"

#include "PowerControl.h"

WorldTime_RxTypedef Control_WorldTime;
uint32_t Robot_FPS;
int init_flag;

void Robot_Control(void const *argument)
{
	portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入数据发送
	

#if(USING_BOARD == BOARD1)
	for(;;)
	{

//云台初始化 这里是为了防止一上电 云台就转动
	if(init_flag == 0)
	{
		Cloud_FUN.Cloud_Init();
		init_flag = 1;
	}
		
		Cloud_FUN.Cloud_Sport_Out();
		//if(M3508_Array[FricL_Wheel].OffLineFlag == 0 && M3508_Array[FricR_Wheel].OffLineFlag == 0 && M3508_Array[FricU_Wheel].OffLineFlag == 0)
	  Shoot_Fun.Shoot_Processing();		
		//发送左右上摩擦轮 拨盘电机 pitch轴电机 yaw轴电机电流
		
		uint8_t data1[8],data2[8],data3[8];
		M3508_FUN.M3508_setCurrent(M3508_Array[FricL_Wheel].outCurrent,
							   M3508_Array[FricR_Wheel].outCurrent,
							   M3508_Array[FricU_Wheel].outCurrent,
						     M3508_Array[Dial_Motor].outCurrent, 
								 data1);

		J4310_Fun.J4310_setParameter(J4310s_Pitch.outPosition,
							   J4310s_Pitch.outSpeed,
							   J4310s_Pitch.outKp,
								 J4310s_Pitch.outKd,
								 J4310s_Pitch.outTorque,
						     data2);
		
		M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent,
								 0,
								 0,
								 0,
								 data3);
								 
		Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan1, FDCAN_STANDARD_ID, M3508_SENDID_Chassis, data1);   //摩擦轮和拨弹轮ID改为了1~4
		Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, J4310_SENDID_Pitch, data2);
	  Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan1, FDCAN_STANDARD_ID, M6020_SENDID, data3);
		
		Get_FPS(&Control_WorldTime, &Robot_FPS);
		vTaskDelayUntil(&xLastWakeTime,TimeIncrement);
	}
#elif(USING_BOARD == BOARD2)
	for(;;)
	{	
		PowerControl_Fun.PowerContol_MsgRenew();
		PowerControl_Fun.PowerControl_MsgSend();
		
		if(init_flag == 0 && M6020s_Yaw.OffLineFlag == 0 && M6020s_Yaw.InfoUpdateFrame != 0)
		{
			Cloud_FUN.Cloud_Init();
			init_flag = 1;
		}
		if(M6020s_Yaw.OffLineFlag == 0 && Saber_Angle.OffLineFlag == 0&& M6020s_Yaw.InfoUpdateFrame != 0 && ControlMes.Check_In_Flag != 1)
			Cloud_FUN.Cloud_Sport_Out();
		if(Saber_Angle.OffLineFlag == 0)
		Mecanum_Fun.Mecanum_Chassis_out();

		Get_FPS(&Control_WorldTime, &Robot_FPS);
		vTaskDelayUntil(&xLastWakeTime,TimeIncrement);	
	}
#endif		
}

