/**
 * @file Task_RobotControl.c
 * @author SJW
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "Task_RobotControl.h"

void Robot_Control(void const *argument)
{

    portTickType xLastWakeTime;                        // 记录任务上次唤醒的时间点，用于精确周期控制。
    xLastWakeTime = xTaskGetTickCount();               // 获取当前系统时钟计数
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); // 每3毫秒强制进入数据发送

    for (;;)
    {

        Cloud_FUN.Remote_Change(); // 变速小陀螺
        Cloud_FUN.Cloud_Sport_Out();
        Shoot_Fun.Shoot_Processing();

        /****************整合电流数据***************/
        uint8_t data1[8], data2[8], data3[8];
        M3508_FUN.M3508_setCurrent(M3508_Array[Fric_Front_1].outCurrent, M3508_Array[Fric_Front_2].outCurrent, M3508_Array[Fric_Front_3].outCurrent, M3508_Array[Dial_Wheel].outCurrent, data1);
        M3508_FUN.M3508_setCurrent(M3508_Array[Fric_Back_1].outCurrent, M3508_Array[Fric_Back_2].outCurrent, M3508_Array[Fric_Back_3].outCurrent, 0, data2);
        J4340_Fun.J4340_setParameter(J4340s_Pitch.outPosition, J4340s_Pitch.outSpeed, J4340s_Pitch.outKp, J4340s_Pitch.outKd, J4340s_Pitch.outTorque, data3);

        /****************发送电流数据***************/
        Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M3508_SENDID_Fric_Dial_1, data1);
        Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, M3508_SENDID_Fric_Dial_2, data2);
        Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, J4340_SENDID_Pitch, data3);
        /****************发送电流数据 end***************/

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);

    }
}
