/**
 * @file Task_CanMsg.c
 * @author why
 * @brief
 * @version 0.1
 * @date 2021-03-30
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Task_FdcanReceive.h"
#include "BSP_Fdcan.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Cloud_Control.h"
#include "BSP_BoardCommunication.h"


/**
  * @Data   2021-03-28
  * @brief  fdcan1接收任务
  * @param  void
  * @retval void
  */
void Fdcan1Receives(void const *argument)
{
    Fdcan_Export_Data_t Fdcan_Export_Data;

    uint32_t ID;
    for (;;)
    {
        xQueueReceive(FDCAN1_ReceiveHandle, &Fdcan_Export_Data, portMAX_DELAY);
        ID = Fdcan_Export_Data.FDCAN_RxHeader.Identifier;
		//接收  摩擦轮电机 反馈数据
		if (ID >= M3508_READID_START && ID <= M3508_READID_END)
        {
            M3508_FUN.M3508_getInfo(Fdcan_Export_Data);
        }
		//接收 拨弹电机 反馈数据
		else if (ID == M2006_DIAL_ID)
        {
            M2006_FUN.M2006_getInfo(Fdcan_Export_Data);
        }
		//接收 云台pitch电机 反馈数据
        else if (ID == M6020_PITCH_ID)
        {
            M6020_Fun.M6020_getInfo(Fdcan_Export_Data);
        }
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
		if (ID == FDCAN_ID_GIMBAL)
        {
            Board1_FUN.Board1_getGimbalInfo(Fdcan_Export_Data);
        }
//		else if(ID == FDCAN_ID_IT_KEYCOMMAND)
//				{
//						Board1_FUN.Board1_getKeycommandInfo(Fdcan_Export_Data);
//				}
//		
	//暂时没用	（图传信息接收）
    }
}
