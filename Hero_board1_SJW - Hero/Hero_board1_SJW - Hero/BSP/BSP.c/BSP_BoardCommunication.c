/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)，ZS
 * @brief
 * @version 2.5
 * @date 2025-1-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "BSP_BoardCommunication.h"

ControlMessge ControlMes;
ext_robot_keycommand_t ext_robot_keycommand;

/*************函数声明***********/
void Board1_To_2(void);
void Board1_getGimbalInfo(Can_Export_Data_t RxMessage);
void Board1_getKeycommandInfo(Can_Export_Data_t RxMessage);
/*************函数声明end***********/

Board1_FUN_t Board1_FUN = Board1_FunGroundInit;

// 此函数用来按照报文规则生成数据并发送。
void Board1_To_2(void)
{
  uint8_t data[8] = {0};
  uint8_t data2_Fun[8] = {0};

  // 打包数据
  data[0] = ControlMes.x_velocity >> 8;
  data[1] = ControlMes.x_velocity;
  data[2] = ControlMes.y_velocity >> 8;
  data[3] = ControlMes.y_velocity;
  data[4] = ControlMes.z_rotation_velocity >> 8;
  data[5] = ControlMes.z_rotation_velocity;
  data[6] = ControlMes.yaw_velocity >> 8;
  data[7] = ControlMes.yaw_velocity;
  // 数据发送
  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_CHASSIS, data);

  data2_Fun[0] = ControlMes.yaw_position >> 8;
  data2_Fun[1] = ControlMes.yaw_position;
  data2_Fun[2] = ControlMes.shoot_Speed; 
  data2_Fun[3] |= (uint8_t)(ControlMes.fric_Flag & 0x01) << 0;
  data2_Fun[3] |= (uint8_t)(ControlMes.AutoAimFlag & 0x01) << 1;
  data2_Fun[3] |= (uint8_t)(ControlMes.change_Flag & 0x01) << 2;
  data2_Fun[3] |= (uint8_t)(ControlMes.modelFlag & 0x01) << 3;
  data2_Fun[4] = M3508_Array[Dial_Wheel].outCurrent >> 8;
  data2_Fun[5] = M3508_Array[Dial_Wheel].outCurrent;
  // 数据发送
  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_GIMBAL, data2_Fun);
}

void Board1_getGimbalInfo(Can_Export_Data_t RxMessage)
{
  ControlMes.yaw_realAngle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
  ControlMes.Blood_Volume = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
  ControlMes.Speed_Bullet = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
  ControlMes.Speed_Bullet /= 1000;
  ControlMes.tnndcolor = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 0) & 0x01;
  ControlMes.game_start = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 1) & 0x01;
}
