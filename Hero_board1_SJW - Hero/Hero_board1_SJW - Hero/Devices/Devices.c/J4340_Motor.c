/**
 * @file J4340_Motor.c
 * @author SJW
 * @brief
 * @version 0.1
 * @date 2025-8-19
 *
 * @copyright Copyright (c)
 *
 */
#include "J4340_Motor.h"

// 直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
J4340s_t J4340s_Pitch;                      // 8
J4340s_t *J4340_Array[1] = {&J4340s_Pitch}; // 对应电机的ID必须为：索引+1
#define J4340_Amount 1

/****************函数声明****************/
void J4340_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
void J4340_Enable(void);
void J4340_Save_Pos_Zero(void);
void J4340_getInfo(Can_Export_Data_t RxMessage);
void J4340_setTargetAngle(J4340s_t *J4340, int32_t angle);
void J4340_Reset(J4340s_t *J4340);
void Check_J4340(void);
/****************函数声明end****************/

J4340_Fun_t J4340_Fun = J4340_FunGroundInit;
#undef J4340_FunGroundInit

/**
 * @brief  uint类型转换为float类型
 * @param
 * @retval None
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

/**
 * @brief  float类型转换为uint类型
 * @param
 * @retval None
 */
static int float_to_uint(float X_float, float X_min, float X_max, int bits)
{
    float span = X_max - X_min;
    float offset = X_min;
    return (int)((X_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  设置J4340电机参数
 * @param  uq1：角度  uq2：转速  uq3：Kp  uq4：Kd  uq5：转矩
 * @retval None
 */
void J4340_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data)
{
    float Postion = uq1 / 8192.0f * 2.0f * Pi;
    uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

    float P_MAX, V_MAX, T_MAX;
    P_MAX = 3.141593f;
    V_MAX = 200.f;
    T_MAX = 7.f;

    Postion_Tmp = float_to_uint(Postion, -P_MAX, P_MAX, 16);
    Velocity_Tmp = float_to_uint(uq2, -V_MAX, V_MAX, 12);
    Torque_Tmp = float_to_uint(uq5, -T_MAX, T_MAX, 12);
    KP_Tmp = float_to_uint(uq3, 0, 500, 12);
    KD_Tmp = float_to_uint(uq4, 0, 5, 12);

    data[0] = (uint8_t)(Postion_Tmp >> 8);
    data[1] = (uint8_t)(Postion_Tmp);
    data[2] = (uint8_t)(Velocity_Tmp >> 4);
    data[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
    data[4] = (uint8_t)(KP_Tmp);
    data[5] = (uint8_t)(KD_Tmp >> 4);
    data[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
    data[7] = (uint8_t)(Torque_Tmp);
}

/**
 * @brief  使能J4340电机
 * @param
 * @retval None
 */
void J4340_Enable()
{
    Can_Send_Data_t Can_Send_Data;

    Can_Send_Data.CAN_TxHeader.StdId = 0x001;
    Can_Send_Data.CAN_TxHeader.IDE = CAN_ID_STD;             // ID类型
    Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA;           // 发送的为数据
    Can_Send_Data.CAN_TxHeader.DLC = 0x08;                   // 数据长度为8字节
    Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE; // 不发送全局时间

    Can_Send_Data.CANx_Send_RxMessage[0] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[1] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[2] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[3] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[4] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[5] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[6] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[7] = 0xFC;

    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, 0x001, Can_Send_Data.CANx_Send_RxMessage);
}
/**
 * @brief  重新设置J4340电机零点
 * @param
 * @retval None
 */
void J4340_Save_Pos_Zero(void)
{
    Can_Send_Data_t Can_Send_Data;

    Can_Send_Data.CAN_TxHeader.StdId = 0x001;
    Can_Send_Data.CAN_TxHeader.IDE = CAN_ID_STD;             // ID类型
    Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA;           // 发送的为数据
    Can_Send_Data.CAN_TxHeader.DLC = 0x08;                   // 数据长度为8字节
    Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE; // 不发送全局时间
    Can_Send_Data.CANx_Send_RxMessage[0] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[1] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[2] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[3] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[4] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[5] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[6] = 0xFF;
    Can_Send_Data.CANx_Send_RxMessage[7] = 0xFE;

    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, 0x001, Can_Send_Data.CANx_Send_RxMessage);
}

/**
 * @brief  从CAN报文中获取J4340电机信息
 * @param  RxMessage 	FDCAN报文接收结构体
 * @retval None
 */

void J4340_getInfo(Can_Export_Data_t RxMessage)
{
    int32_t StdId;
    StdId = (RxMessage.CANx_Export_RxMessage[0]) & 0x0F;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - J4340_READID_PITCH; // 由0开始
    float P_MAX, V_MAX, T_MAX;

    P_MAX = 12.5f;
    V_MAX = 200.f;
    T_MAX = 7.f; // J4340电机数据

    J4340_Array[StdId]->lastAngle = J4340_Array[StdId]->realAngle;
    J4340_Array[StdId]->state = RxMessage.CANx_Export_RxMessage[0] >> 4;
    J4340_Array[StdId]->angleInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[1] << 8) | RxMessage.CANx_Export_RxMessage[2]);
    J4340_Array[StdId]->speedInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[3] << 4) | (RxMessage.CANx_Export_RxMessage[4] >> 4));
    J4340_Array[StdId]->torqueInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[4] & 0xF << 8) | RxMessage.CANx_Export_RxMessage[5]);
    J4340_Array[StdId]->realAngle = uint_to_float(J4340_Array[StdId]->angleInit, -P_MAX, P_MAX, 16);
    J4340_Array[StdId]->realAngle = J4340_Array[StdId]->realAngle / 2.0f * 3.1415927f * 36.0f;
    J4340_Array[StdId]->realSpeed = uint_to_float(J4340_Array[StdId]->speedInit, -V_MAX, V_MAX, 12);
    J4340_Array[StdId]->torque = uint_to_float(J4340_Array[StdId]->torqueInit, -T_MAX, T_MAX, 12);
    J4340_Array[StdId]->temperatureMOS = (float)(RxMessage.CANx_Export_RxMessage[6]);
    J4340_Array[StdId]->temperatureRotor = (float)(RxMessage.CANx_Export_RxMessage[7]);

    if (J4340_Array[StdId]->realAngle - J4340_Array[StdId]->lastAngle < -6500)
    {
        J4340_Array[StdId]->turnCount++;
    }

    if (J4340_Array[StdId]->lastAngle - J4340_Array[StdId]->realAngle < -6500)
    {
        J4340_Array[StdId]->turnCount--;
    }

    J4340_Array[StdId]->totalAngle = J4340_Array[StdId]->realAngle + (8192 * J4340_Array[StdId]->turnCount);
    // 帧率统计，数据更新标志位
    J4340_Array[StdId]->InfoUpdateFrame++;
    J4340_Array[StdId]->InfoUpdateFlag = 1;
}

/*
 *@brief  设定J4340电机的目标角度
 * @param  motorName 	电机名字 @ref M6623Name_e
 *			angle
 * @retval None
 * */
void J4340_setTargetAngle(J4340s_t *J4340, int32_t angle)
{
    J4340->targetAngle = angle;
}

/*************************************
 * Method:    J4340_OverflowReset
 * Returns:   void
 * 说明：调运此函数以解决totalAngle 等溢出的问题。
 ************************************/
void J4340_Reset(J4340s_t *J4340)
{
    J4340->lastAngle = J4340->realAngle;
    J4340->totalAngle = J4340->realAngle;
    J4340->turnCount = 0;
}

/**
 * @brief 4340检测
 *
 */
void Check_J4340(void)
{
#if (USING_BOARD == BOARD2)
    // J4340检测
    for (int i = 0; i < 1; i++)
    {
        if (J4340_Array[i]->InfoUpdateFrame < 1)
        {
            J4340_Array[i]->OffLineFlag = 1;
        }
        else
        {
            J4340_Array[i]->OffLineFlag = 0;
        }
        J4340_Array[i]->InfoUpdateFrame = 0;
    }
#else
    for (int i = 1; i < 2; i++)
    {
        if (J4340_Array[i]->InfoUpdateFrame < 1)
        {
            J4340_Array[i]->OffLineFlag = 1;
        }
        else
        {
            J4340_Array[i]->OffLineFlag = 0;
        }
        J4340_Array[i]->InfoUpdateFrame = 0;
    }
#endif
}
