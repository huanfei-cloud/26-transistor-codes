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

     if (ID == M2006_J2_ID)
     {
      get_motor_data(&M2006s_J2,Can_Export_Data);
     }
		 else if (ID == M2006_J3_ID)
     {
      get_motor_data(&M2006s_J2,Can_Export_Data);
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
//  Can_Export_Data_t Can_Export_Data;
//  uint32_t ID;
//  for (;;)
//  {
//    xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
//    ID = Can_Export_Data.CAN_RxHeader.StdId;

//  }
}
