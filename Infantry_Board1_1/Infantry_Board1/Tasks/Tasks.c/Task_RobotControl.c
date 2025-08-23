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
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ3����ǿ�ƽ������ݷ���
    for (;;)
    {
			
			Cloud_FUN.Remote_Change();     //����С����
			Cloud_FUN.Cloud_Sport_Out();   //��������ͳһ��shoot���淢������cloud������
			Shoot_Fun.Shoot_Processing();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
