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
#include "DM_Motor.h"
#include "BSP_Can.h"
#include "stm32f4xx_hal_can.h"

//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用
DM_Motors_t J4340s_J3;   //ID为0x04
DM_Motors_t J4310s_J4;   //ID为0x05
DM_Motors_t J4310s_J5;   //ID为0x06
DM_Motors_t J4310s_J6;   //ID为0x07
DM_Motors_t *DM_Array[totalnum] = {&J4340s_J3,&J4310s_J4,&J4310s_J5,&J4310s_J6};

/********数据声明********/
DM_Motors_Fun_t DM_Motors_Fun = DM_FunGroundInit;
#undef DM_FunGroundInit

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
  * @brief  设置达妙电机参数（id号为8）
  * @param  uq1：角度  uq2：转速  uq3：Kp  uq4：Kd  uq5：转矩
  * @retval None
  */
void DM_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5,float p_max,float v_max,float t_max,uint8_t *data)
{
  float Postion = uq1 / 8192 * 2 * Pi;
  uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

  float P_MAX, V_MAX, T_MAX;
  P_MAX = p_max;
  V_MAX = v_max;
  T_MAX = t_max;

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
 * @brief  使能达妙电机
 * @param
 * @retval None
 */
void DM_Enable(uint32_t id,CAN_HandleTypeDef *hcan)
{
  Can_Send_Data_t Can_Send_Data;
		
  Can_Send_Data.CAN_TxHeader.StdId = id;
  Can_Send_Data.CAN_TxHeader.IDE = CAN_ID_STD;             // ID类型
  Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA;           // 发送的是数据
  Can_Send_Data.CAN_TxHeader.DLC = 0x08;                   // 8字节
  Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE; 

  Can_Send_Data.CANx_Send_RxMessage[0] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[1] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[2] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[3] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[4] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[5] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[6] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[7] = 0xFC;
	
	Can_Fun.CAN_SendData(CAN_SendHandle, hcan, CAN_ID_STD, id, Can_Send_Data.CANx_Send_RxMessage);
}


/**
 * @brief  MIT模式下电机参数初始化
 * @param  uq1：扭矩  uq2;角度 uq3：转速  uq4：Kp  uq5：Kd
 * @retval None
 */
void DM_MIT_Init(DM_Motors_t *DMmotor,float uq1,float uq2, float uq3, float uq4, float uq5)
{
	DMmotor->outTorque = uq1;
	DMmotor->outPosition = uq2;
	DMmotor->outSpeed = uq3;
	DMmotor->outKp = uq4;
	DMmotor->outKd = uq5;
}

/**
 * @brief  重新设置达妙电机零点
 * @param
 * @retval None
 */
void DM_Save_Pos_Zero(uint32_t id,CAN_HandleTypeDef *hcan)
{
  Can_Send_Data_t Can_Send_Data;
	uint32_t TxMailbox;

  Can_Send_Data.CAN_TxHeader.StdId = id;
  Can_Send_Data.CAN_TxHeader.IDE = CAN_ID_STD;             // ID类型
  Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA;           // 发送的是数据
  Can_Send_Data.CAN_TxHeader.DLC = 0x08;                   // 8字节
  Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE; 
//	Fdcan_Send_Data.FDCAN_TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;  //发送节点处于主动错误状态
//	Fdcan_Send_Data.FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        //不转换波特率
//	Fdcan_Send_Data.FDCAN_TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;        //经典CAN模式
//	Fdcan_Send_Data.FDCAN_TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
//	Fdcan_Send_Data.FDCAN_TxHeader.MessageMarker = 0;

  Can_Send_Data.CANx_Send_RxMessage[0] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[1] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[2] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[3] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[4] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[5] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[6] = 0xFF;
  Can_Send_Data.CANx_Send_RxMessage[7] = 0xFE;

   HAL_CAN_AddTxMessage(hcan, &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage, &TxMailbox);
//  HAL_CAN_AddMessageToTxFifoQ(&hcan2, &Can_Send_Data.CAN_TxHeader, Can_Send_Data.CANx_Send_RxMessage);
}

/**
  * @brief  从CAN报文中获取达妙电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @param  model       因为ID冲突原因，DM电机ID不连续，加上此变量使能用一个数组表示
  * @retval None
  */

void DM_getInfo(Can_Export_Data_t RxMessage,float p_max,float v_max,float t_max)
{
  int32_t StdId;
  StdId = (RxMessage.CANx_Export_RxMessage[0]) & 0x0F;
   
	StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - RECEIVE_START_ID; //由零开始
	
  float P_MAX, V_MAX, T_MAX;

  P_MAX = p_max;
  V_MAX = v_max;
  T_MAX = t_max; // 达妙电机数据

  DM_Array[StdId]->lastAngle = DM_Array[StdId]->realAngle;
  DM_Array[StdId]->state = RxMessage.CANx_Export_RxMessage[0] >> 4;
  DM_Array[StdId]->angleInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[1] << 8) | RxMessage.CANx_Export_RxMessage[2]);
  DM_Array[StdId]->speedInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[3] << 4) | (RxMessage.CANx_Export_RxMessage[4] >> 4));
  DM_Array[StdId]->torqueInit = (uint16_t)((RxMessage.CANx_Export_RxMessage[4] & 0xF << 8) | RxMessage.CANx_Export_RxMessage[5]);
  DM_Array[StdId]->realAngle = uint_to_float(DM_Array[StdId]->angleInit, -P_MAX, P_MAX, 16);
  DM_Array[StdId]->realAngle = DM_Array[StdId]->realAngle *409.6f + 4096.0f;
  DM_Array[StdId]->realSpeed = uint_to_float(DM_Array[StdId]->speedInit, -V_MAX, V_MAX, 12);
  DM_Array[StdId]->torque = uint_to_float(DM_Array[StdId]->torqueInit, -T_MAX, T_MAX, 12);
  DM_Array[StdId]->temperatureMOS = (float)(RxMessage.CANx_Export_RxMessage[6]);
  DM_Array[StdId]->temperatureRotor = (float)(RxMessage.CANx_Export_RxMessage[7]);

  if (DM_Array[StdId]->realAngle - DM_Array[StdId]->lastAngle < -6500)
  {
    DM_Array[StdId]->turnCount++;
  }

  if (DM_Array[StdId]->lastAngle - DM_Array[StdId]->realAngle < -6500)
  {
    DM_Array[StdId]->turnCount--;
  }

  DM_Array[StdId]->totalAngle = DM_Array[StdId]->realAngle + (8192 * DM_Array[StdId]->turnCount);
  //帧率统计，数据更新标志位
  DM_Array[StdId]->InfoUpdateFrame++;
  DM_Array[StdId]->InfoUpdateFlag = 1;
}

/**
  * @brief  设定达妙电机的目标角度
  * @param  DMmotor  电机数据结构体地址 
  *         angle	 机械角度值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
  * @retval None
  */
void DM_setTargetAngle(DM_Motors_t *DMmotor, int32_t angle)
{
  DMmotor->targetAngle = angle;
}

/**
  * @brief  设定达妙电机的目标角度
  * @param  DMmotor  电机数据结构体地址 
  *         angle	 机械角度值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
  * @retval None
  * 说明：调运此函数以解决totalAngle 等溢出的问题
  */
void DM_Reset(DM_Motors_t *DMmotor)
{
  //解包数据
  DMmotor->lastAngle = DMmotor->realAngle;
  DMmotor->totalAngle = DMmotor->realAngle;
  DMmotor->turnCount = 0;
}

/**
 * @brief 达妙电机检测
 *
 */
void Check_DM(void)
{
#if (USING_BOARD == BOARD2)
  // 达妙电机检测
  for (int i = 0; i < 1; i++)
  {
    if (DM_Array[i]->InfoUpdateFrame < 1)
    {
      DM_Array[i]->OffLineFlag = 1;
    }
    else
    {
      DM_Array[i]->OffLineFlag = 0;
    }
    DM_Array[i]->InfoUpdateFrame = 0;
  }
#else
  for (int i = 1; i < 2; i++)
  {
    if (DM_Array[i]->InfoUpdateFrame < 1)
    {
      DM_Array[i]->OffLineFlag = 1;
    }
    else
    {
      DM_Array[i]->OffLineFlag = 0;
    }
    DM_Array[i]->InfoUpdateFrame = 0;
  }
#endif
}

/**
 * @brief  通过位置环得到目标速度
* @param  0-8192
 * @retval None
 */
void DMmotor_location_change(DM_Motors_t *motor, enum pid_control model, float target,float real)
{

	if (model == pid_control1)
	{
		motor->targetSpeed+= (int16_t)PID_Model1_Update(&motor->l_pid_object, real, target);
	}
	if (model == pid_control2)
	{
		motor->targetSpeed = (int16_t)PID_Model2_Update(&motor->l_pid_object, real, target);
	}
	if (model == pid_control3)
	{
		motor->targetSpeed = (int16_t)PID_Model3_Update(&motor->l_pid_object, real, target);
	}

}
