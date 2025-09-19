/**
 * @file BSP_Fdcan.h
 * @author ZS (2729511164@qq)
 * @brief Init of FDCAN
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _BSP_FDCAN
#define _BSP_FDCAN

#include "fdcan.h"
#include "cmsis_os.h"
#include "queue.h"
#include "typedef.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Fdcan1_Type 1
#define Fdcan2_Type 2

#define Fdcan_FunGroundInit           \
    {                               \
        &FDCAN_IT_Init,               \
        &FDCAN_SendData,              \
    }

#define Fdcan_DataGroundInit \
    {                      \
        {0}, {0},          \
    }

typedef struct
{
    struct
    {
        FDCAN_FilterTypeDef FDCAN_Filter;
    } FDCAN_FilterTypedef;

    struct
    {
        FDCAN_RxHeaderTypeDef FDCANx_RxHeader;
        uint8_t FDCAN_RxMessage[8];
    } FDCAN_RxTypedef;

} Fdcan_Data_t;

typedef struct
{
    void (*FDCAN_IT_Init)(FDCAN_HandleTypeDef *hfdcanx, uint8_t Fdcan_type);
    void (*FDCAN_SendData)(osMessageQId FDCANx_Handle, FDCAN_HandleTypeDef *FDCANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
} Fdcan_Fun_t;

extern Fdcan_Fun_t Fdcan_Fun;
#endif /*__BSP_FDCAN*/
