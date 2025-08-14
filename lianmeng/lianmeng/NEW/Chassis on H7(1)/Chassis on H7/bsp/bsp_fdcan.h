/**
 * @file bsp_fdcan.h
 * @author xyz
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 1.1
 * @date 2024-09-13
 * @copyright Transistor BUAA
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "QueueHandle.h"
#include "fdcan.h"

/* Exported macros -----------------------------------------------------------*/

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

// // 滤波器编号
// #define CAN_FILTER(x) ((x) << 3)

// // 接收队列
// #define CAN_FIFO_0 (0 << 2)
// #define CAN_FIFO_1 (1 << 2)

// // 标准帧或扩展帧   决定IDE位
// #define CAN_STDID (0 << 1)
// #define CAN_EXTID (1 << 1)

// // 数据帧或遥控帧  决定RTR位
// #define CAN_DATA_TYPE (0 << 0)
// #define CAN_REMOTE_TYPE (1 << 0)

/* Exported types ------------------------------------------------------------*/


/**
 * @brief CAN接收的信息结构体
 *
 */
struct Struct_CAN_Rx_Buffer
{ 
    //header包括了  标准帧/拓展帧   数据帧/遥控帧   数据长度 等因素
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

struct Struct_CAN_Tx_Buffer
{
	FDCAN_TxHeaderTypeDef Header;
	uint8_t Data[8];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(struct Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信接收处理结构体
 *
 */
struct Struct_CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    struct Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

extern QueueHandle_t FDCAN_SendHandle;   //extern RTOS handle

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern struct Struct_CAN_Manage_Object CAN1_Manage_Object;
extern struct Struct_CAN_Manage_Object CAN2_Manage_Object;

// 用于向电调发送控制指令控制电调的电流输出
extern uint8_t CAN_0x1ff_Tx_Data[];
extern uint8_t CAN_0x200_Tx_Data[];
extern uint8_t CAN_0x2ff_Tx_Data[];

void bsp_can_init(void);

void can_filter_init(void);

void bsp_fdcan_set_baud(FDCAN_HandleTypeDef *hfdcan, uint8_t mode, uint8_t baud);

uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf);


/* Exported function declarations ---------------------------------------------*/

#endif


