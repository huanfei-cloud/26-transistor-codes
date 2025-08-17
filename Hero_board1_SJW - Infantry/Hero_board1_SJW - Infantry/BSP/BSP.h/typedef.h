/**
 * @file typedef.h
 * @author Why (1091537104@qq)
 * @brief
 * @version 0.1
 * @date 2023-08-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include "can.h"
#include <stdint.h>

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

/*************CAN对外数据接口************/
typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CANx_Export_RxMessage[8];
} Can_Export_Data_t;

/*************CAN发送数据接口************/
typedef struct
{
    CAN_HandleTypeDef *Canx;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t CANx_Send_RxMessage[8];
} Can_Send_Data_t;

#endif /* __TYPEDEFS_H */
