/**
 * @file M2006_Motor.c
 * @author Why
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "M2006_Motor.h"

/* M3508所有电机的数组 */
M2006s_t M2006_Array[Total_Num] = {
    [Dial_Motor] = {0},
};

void M2006_getInfo(Fdcan_Export_Data_t RxMessage);
void M2006_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);

M2006_FUN_t M2006_FUN = M2006_FunGroundInit;
#undef M2006_FunGroundInit
/**
  * @brief  设置M2006电机电流值（id号为7）M2006与6623共用发送函数
  * @param  iqx (x:5) 对应id号电机的电流值，范围-10000~0~10000
  * @retval None
  */
void M2006_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data)
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
  * @brief  从CAN报文中获取M2006电机信息
  * @param[in]  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M2006_getInfo(Fdcan_Export_Data_t RxMessage)
{
    int32_t StdId;
    StdId = (int32_t)RxMessage.FDCAN_RxHeader.Identifier - M2006_READID_START;

    M2006_Array[StdId].lastAngle = M2006_Array[StdId].realAngle;
    //解包数据，数据格式详见C610电调说明书P9
    M2006_Array[StdId].realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    M2006_Array[StdId].realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    M2006_Array[StdId].realTorque = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);

    if (M2006_Array[StdId].realAngle - M2006_Array[StdId].lastAngle < -6000)
    {
        M2006_Array[StdId].turnCount++;
    }
    else if (M2006_Array[StdId].lastAngle - M2006_Array[StdId].realAngle < -6000)
    {
        M2006_Array[StdId].turnCount--;
    }
    M2006_Array[StdId].totalAngle = M2006_Array[StdId].realAngle + (8192 * M2006_Array[StdId].turnCount);
    M2006_Array[StdId].lastAngle = M2006_Array[StdId].realAngle;
    M2006_Array[StdId].InfoUpdateFrame++;
    M2006_Array[StdId].InfoUpdateFlag = 1;
}

/**
* @brief  对电机参数进行初始化
  * @param  None
  * @retval None
  * @attention 当前只针对其初始输出电流进行了清零，以后有更多需求需改写
  */
void M2006_Init(void)
{
    for(int i = 0 ; i <= 3 ; i++ )
    {
        M2006_Array[i].outCurrent = 0;
    }
}
