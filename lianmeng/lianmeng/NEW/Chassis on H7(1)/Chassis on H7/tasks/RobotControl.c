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

#include "RobotControl.h"
#include "PowerControl.h"
#include "chassis.h"
#include "DT7.h"
#include "BoardCommunication.h"
#include "PowerBank.h"
#include "IMU.h"

void Robot_Control(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(5); // 每2毫秒强制进入数据发送

  for (;;)
  {
    DT7_Handle();
		IMU_Handler();
    chassis_task(&chassis_control);
    Board2_FUN.Board2_To_1();
    // PowerControl_Fun.PowerControl_MsgSend();
		PowerBankSend();
		
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
}
