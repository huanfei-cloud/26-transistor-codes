/**
 * @file J4310_Motor.c
 * @author ZS (2729511164@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c)
 *
 */
#include "J4310_Motor.h"

// 直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
J4310s_t J4310s_Pitch;                      // 8
J4310s_t *J4310_Array[1] = {&J4310s_Pitch}; // 对应电机的ID必须为：索引+1
#define J4310_Amount 1
void J4310_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
void J4310_Enable(void);
void J4310_Save_Pos_Zero(void);
void J4310_getInfo(Can_Export_Data_t RxMessage);
void J4310_setTargetAngle(J4310s_t *J4310, int32_t angle);
void J4310_Reset(J4310s_t *J4310);
void Check_J4310(void);

J4310_Fun_t J4310_Fun = J4310_FunGroundInit;
#undef J4310_FunGroundInit

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
 * @brief  设置J4310电机参数（id号为8）
 * @param  uq1：角度  uq2：转速  uq3：Kp  uq4：Kd  uq5：转矩
 * @retval None
 */
void J4310_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data)
{
  float Postion = uq1 / 8192 * 2 * Pi;
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
 * @brief  使能J4310电机
 * @param
 * @retval None
 */
void J4310_Enable()
{
  Can_Send_Data_t Can_Send_Data;
  uint32_t TxMailbox;

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

  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, 0x001, Can_Send_Data.CANx_Send_RxMessage);
  // HAL_CAN_AddTxMessage(&hcan2,  &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage, &TxMailbox);
}
/**
 * @brief  重新设置J4310电机零点
 * @param
 * @retval None
 */
void J4310_Save_Pos_Zero(void)
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

  //  HAL_CAN_AddMessageToTxFifoQ(&hcan2, &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage);
}

/**
 * @brief  从FDCAN报文中获取J4310电机信息
 * @param  RxMessage 	FDCAN报文接收结构体
 * @retval None
 */

void J4310_getInfo(Can_Export_Data_t RxMessage)
{
  int32_t StdId;
  StdId = (RxMessage.CANx_Export_RxMessage[0]) & 0x0F;
  StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - J4310_READID_PITCH; // 由0开始
  float P_MAX, V_MAX, T_MAX;

  P_MAX = 12.5f;
  V_MAX = 200.f;
  T_MAX = 7.f; // J4310电机数据

  J4310_Array[StdId]->lastAngle = J4310_Array[StdId]->realAngle;
  J4310_Array[StdId]->state = RxMessage.CANx_Export_RxMessage[0] >> 4;
  J4310_Array[StdId]->angleInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[1] << 8) | RxMessage.CANx_Export_RxMessage[2]);
  J4310_Array[StdId]->speedInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[3] << 4) | (RxMessage.CANx_Export_RxMessage[4] >> 4));
  J4310_Array[StdId]->torqueInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[4] & 0xF << 8) | RxMessage.CANx_Export_RxMessage[5]);
  J4310_Array[StdId]->realAngle = uint_to_float(J4310_Array[StdId]->angleInit, -P_MAX, P_MAX, 16);
  J4310_Array[StdId]->realAngle = J4310_Array[StdId]->realAngle / 2 * 3.1415927 * 36.f;
  J4310_Array[StdId]->realSpeed = uint_to_float(J4310_Array[StdId]->speedInit, -V_MAX, V_MAX, 12);
  J4310_Array[StdId]->torque = uint_to_float(J4310_Array[StdId]->torqueInit, -T_MAX, T_MAX, 12);
  J4310_Array[StdId]->temperatureMOS = (float)(RxMessage.CANx_Export_RxMessage[6]);
  J4310_Array[StdId]->temperatureRotor = (float)(RxMessage.CANx_Export_RxMessage[7]);

  if (J4310_Array[StdId]->realAngle - J4310_Array[StdId]->lastAngle < -6500)
  {
    J4310_Array[StdId]->turnCount++;
  }

  if (J4310_Array[StdId]->lastAngle - J4310_Array[StdId]->realAngle < -6500)
  {
    J4310_Array[StdId]->turnCount--;
  }

  J4310_Array[StdId]->totalAngle = J4310_Array[StdId]->realAngle + (8192 * J4310_Array[StdId]->turnCount);
  // 帧率统计，数据更新标志位
  J4310_Array[StdId]->InfoUpdateFrame++;
  J4310_Array[StdId]->InfoUpdateFlag = 1;
}

/*
 *@brief  设定J4310电机的目标角度
 * @param  motorName 	电机名字 @ref M6623Name_e
 *			angle		电流值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
 * @retval None
 * */
void J4310_setTargetAngle(J4310s_t *J4310, int32_t angle)
{
  J4310->targetAngle = angle;
}

/*************************************
 * Method:    J4310_OverflowReset
 * Returns:   void
 * 说明：调运此函数以解决totalAngle 等溢出的问题。
 ************************************/
void J4310_Reset(J4310s_t *J4310)
{
  // 解包数据，数据格式详见C620电调说明书P33
  J4310->lastAngle = J4310->realAngle;
  J4310->totalAngle = J4310->realAngle;
  J4310->turnCount = 0;
}

/**
 * @brief 4310检测
 *
 */
void Check_J4310(void)
{
#if (USING_BOARD == BOARD2)
  // J4310检测
  for (int i = 0; i < 1; i++)
  {
    if (J4310_Array[i]->InfoUpdateFrame < 1)
    {
      J4310_Array[i]->OffLineFlag = 1;
    }
    else
    {
      J4310_Array[i]->OffLineFlag = 0;
    }
    J4310_Array[i]->InfoUpdateFrame = 0;
  }
#else
  for (int i = 1; i < 2; i++)
  {
    if (J4310_Array[i]->InfoUpdateFrame < 1)
    {
      J4310_Array[i]->OffLineFlag = 1;
    }
    else
    {
      J4310_Array[i]->OffLineFlag = 0;
    }
    J4310_Array[i]->InfoUpdateFrame = 0;
  }
#endif
}
