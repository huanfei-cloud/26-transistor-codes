/**
 * @file Task_FdcanMsg.c
 * @author why,ZS
 * @brief
 * @version 0.1
 * @date 2021-03-30
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Task_FdcanReceive.h"
#include "BSP_FDCAN.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Omni_Chassis.h"
#include "Mecanum_Chassis.h"
#include "Cloud_Control.h"
#include "BSP_BoardCommunication.h"
#include "PowerControl.h"
/**
  * @Data   2021-03-28
  * @brief  fdcan1接收任务
  * @param  void
  * @retval void
  */
int16_t J4310_RxID;
	
void Fdcan1Receives(void const *argument)
{
    Fdcan_Export_Data_t Fdcan_Export_Data;

    uint32_t ID;
    for (;;)
    {
        xQueueReceive(FDCAN1_ReceiveHandle, &Fdcan_Export_Data, portMAX_DELAY);
        ID = Fdcan_Export_Data.FDCAN_RxHeader.Identifier;
#if(USING_BOARD == BOARD1 )

        //接收云台 yaw轴电机 反馈数据
        if (ID == M6020_READID_YAW)
        {
            M6020_Fun.M6020_getInfo(Fdcan_Export_Data);
        }
				else if (ID >= M3508_READID_CHASSIS_START && ID <= M3508_READID_CHASSIS_END)
        {
            M3508_FUN.M3508_getInfo(Fdcan_Export_Data);
        }
        //接收 底盘电机  摩擦轮电机 拨盘电机 反馈数据


#elif(USING_BOARD == BOARD2)
        //接收 底盘电机  摩擦轮电机 拨盘电机 反馈数据
        if (ID >= M3508_READID_CHASSIS_START && ID <= M3508_READID_CHASSIS_END)
        {
            M3508_FUN.M3508_getInfo(Fdcan_Export_Data);
        }
        //接收云台 yaw轴电机 pitch电机 反馈数据
        else if (ID == M6020_READID_YAW)
        {
            M6020_Fun.M6020_getInfo(Fdcan_Export_Data);
        }
		else if(ID == 0X30)
		{
			PowerControl_Fun.PowerControl_MsgRec(Fdcan_Export_Data);
		}
#endif


    }
}

/**
  * @Data   2021-03-28
  * @brief  fdcan2接收任务
  * @param  void
  * @retval void
  */
void Fdcan2Receives(void const *argument)
{
    Fdcan_Export_Data_t Fdcan_Export_Data;
    uint32_t ID;
    for (;;)
    {
        xQueueReceive(FDCAN2_ReceiveHandle, &Fdcan_Export_Data, portMAX_DELAY);
        ID = Fdcan_Export_Data.FDCAN_RxHeader.Identifier;
				J4310_RxID = (Fdcan_Export_Data.FDCANx_Export_RxMessage[0])&0x0F;
#if(USING_BOARD == BOARD1)
		if (ID == FDCAN_ID_SABER)   
    {
			Board1_FUN.Board1_getSaberInfo(Fdcan_Export_Data);
    }

		else if (ID == FDCAN_ID_SHOOT_HEAT)
		{
			Board1_FUN.Board1_getShootHeatInfo(Fdcan_Export_Data);
		}
    else if (J4310_RxID == J4310_READID_PITCH )  
    {
			J4310_Fun.J4310_getInfo(Fdcan_Export_Data);
    }
#endif
#if(USING_BOARD == BOARD2)
        //Board2接受Board1数据
        if (ID == FDCAN_ID_CHASSIS)
        {
            Board2_FUN.Board2_getChassisInfo(Fdcan_Export_Data);
        }
        else if (ID == FDCAN_ID_GIMBAL)
        {
            Board2_FUN.Board2_getGimbalInfo(Fdcan_Export_Data);
        }
#endif
		
    }
}
