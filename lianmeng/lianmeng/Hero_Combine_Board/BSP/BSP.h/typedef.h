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

#include "fdcan.h"
#include <stdint.h>


#define u8 uint8_t 
#define u16 uint16_t 
#define u32 uint32_t 

/*************FDCAN�������ݽӿ�************/
typedef struct
{
    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
    uint8_t FDCANx_Export_RxMessage[8];
} Fdcan_Export_Data_t;

/*************CAN�������ݽӿ�************/
typedef struct
{
    FDCAN_HandleTypeDef *Fdcanx;
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
    uint8_t FDCANx_Send_RxMessage[8];
} Fdcan_Send_Data_t;

/**********ϵͳʱ��������ݽӿ�************/
typedef struct
{
    uint32_t WorldTime;      //����ʱ��
    uint32_t Last_WorldTime; //��һ������ʱ��
} WorldTime_RxTypedef;
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);

/************��ʱȫ�ַ��ʱ���***************/
extern uint32_t Robot_FPS;
extern uint32_t GetData_FPS;

#endif /* __TYPEDEFS_H */
