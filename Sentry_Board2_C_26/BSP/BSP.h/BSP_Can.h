/**
 * @file BSP_Can.h
 * @author Why (1091537104@qq)
 * @brief Init of CAN
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _BSP_CAN
#define _BSP_CAN

#include "can.h"
#include "cmsis_os.h"
#include "queue.h"
#include "typedef.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Can1_Type 1
#define Can2_Type 2

#define Can_FunGroundInit           \
    {                               \
        &CAN_IT_Init,               \
        &CAN_SendData,              \
    }

#define Can_DataGroundInit \
    {                      \
        {0}, {0},          \
    }

typedef struct
{
    struct
    {
        CAN_FilterTypeDef CAN_Filter;
    } CAN_FilterTypedef;

    struct
    {
        CAN_RxHeaderTypeDef CANx_RxHeader;
        uint8_t CAN_RxMessage[8];
    } CAN_RxTypedef;

} Can_Data_t;

typedef struct
{
    void (*CAN_IT_Init)(CAN_HandleTypeDef *hcanx, uint8_t Can_type);
    void (*CAN_SendData)(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
} Can_Fun_t;

extern Can_Fun_t Can_Fun;

//用于向电调发送控制命令控制电调的电流输出
extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
#endif /*__BSP_CAN*/
