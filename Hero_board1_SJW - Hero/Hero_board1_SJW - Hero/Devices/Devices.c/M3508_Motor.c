/**
 * @file M3508_Motor.c
 * @author Why
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "M3508_Motor.h"

/************3508电机结构体数组初始声明**********/
M3508s_t M3508_Array[TotalNum] = {
    [Fric_Front_1] = {0},
    [Fric_Front_2] = {0},
    [Fric_Front_3] = {0},
    [Dial_Wheel] = {0},
    [Fric_Back_1] = {0},
    [Fric_Back_2] = {0},
    [Fric_Back_3] = {0},
};

void M3508_getInfo(Can_Export_Data_t RxMessage);
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage);
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);

M3508_FUN_t M3508_FUN = M3508_FunGroundInit;
#undef M3508_FunGroundInit

/**
 * @brief  3508电机参数整合
 * @param  iqx (x:1~4)
 * @retval None
 */
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data)
{
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;
}

/**
 * @brief  从can中获取电机信息
 * @param  RxMessage
 * @retval None
 */
void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x201);

    M3508_Array[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508_Array[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508_Array[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508_Array[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M3508_Array[StdId].realAngle - M3508_Array[StdId].lastAngle < -6000)
    {
        M3508_Array[StdId].turnCount++;
    }
    else if (M3508_Array[StdId].lastAngle - M3508_Array[StdId].realAngle < -6000)
    {
        M3508_Array[StdId].turnCount--;
    }
    M3508_Array[StdId].totalAngle = M3508_Array[StdId].realAngle + (8192 * M3508_Array[StdId].turnCount);
    M3508_Array[StdId].lastAngle = M3508_Array[StdId].realAngle;

    M3508_Array[StdId].InfoUpdateFrame++;
    M3508_Array[StdId].InfoUpdateFlag = 1;
}

/**
 * @brief  电机初始化
 * @param  None
 * @retval None
 * @attention
 */
void M3508_Init(void)
{
    for (int i = 0; i <= 7; i++)
    {
        M3508_Array[i].outCurrent = 0;
    }
}
