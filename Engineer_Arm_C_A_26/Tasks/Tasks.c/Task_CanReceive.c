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
#include "Task_CanReceive.h"

/**
 * @Data   2021-03-28
 * @brief  can1接收任务
 * @param  void
 * @retval void
 */
void Can1Receives(void const *argument)
{
  Can_Export_Data_t Can_Export_Data;

  uint32_t ID;
  for (;;)
  {
    xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
    ID = Can_Export_Data.CAN_RxHeader.StdId;

    // 接收J3电机 J4电机 反馈数据
		if (ID == J4340_READID_J3)
    {
      DM_getInfo(Can_Export_Data,J4340_MaxP,J4340_MaxV,J4340_MaxT);
    }
    else if (ID == J4310_READID_J4)
    {
      DM_getInfo(Can_Export_Data,J4310_MaxP,J4310_MaxV,J4310_MaxT);
    }

  }
}

/**
 * @Data   2021-03-28
 * @brief  can2接收任务
 * @param  void
 * @retval void
 */
void Can2Receives(void const *argument)
{
  Can_Export_Data_t Can_Export_Data;
  uint32_t ID;
  for (;;)
  {
    xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
    ID = Can_Export_Data.CAN_RxHeader.StdId;
		
    // 接收J5电机 J6电机 反馈数据
		if (ID == J4310_READID_J5)
    {
      DM_getInfo(Can_Export_Data,J4310_MaxP,J4310_MaxV,J4310_MaxT);
    }
    else if (ID == J4310_READID_J6)
    {
      DM_getInfo(Can_Export_Data,J4310_MaxP,J4310_MaxV,J4310_MaxT);
    }
  }
}
