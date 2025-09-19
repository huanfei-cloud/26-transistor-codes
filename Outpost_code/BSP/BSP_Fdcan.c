#include "BSP_Fdcan.h"

// 定义全局变量
motor_measure_t motor_chassis[7];  // 存储7个电机的测量数据

// FDCAN 句柄定义
extern FDCAN_HandleTypeDef hfdcan1; // FDCAN1 句柄
extern FDCAN_HandleTypeDef hfdcan2; // FDCAN2 句柄

/**
 * @brief FDCAN 初始化函数
 */
void FDCAN_Init(void)
{
    // 配置 FDCAN1 过滤器
    FDCAN_FilterTypeDef fdcan1_filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0x0000,
        .FilterID2 = 0x0000
    };
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan1_filter) ;
    
    
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                                    FDCAN_REJECT, 
                                    FDCAN_REJECT, 
                                    FDCAN_FILTER_REMOTE, 
                                    FDCAN_FILTER_REMOTE) ;
   
    
    

    
    HAL_FDCAN_ActivateNotification(&hfdcan1, 
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 
                                      0) ;
    HAL_FDCAN_Start(&hfdcan1) ;
    // 配置 FDCAN2 过滤器
    FDCAN_FilterTypeDef fdcan2_filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 14,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0x0000,
        .FilterID2 = 0x0000
    };
    
   HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan2_filter) ;
    
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, 
                                    FDCAN_REJECT, 
                                    FDCAN_REJECT, 
                                    FDCAN_FILTER_REMOTE, 
                                    FDCAN_FILTER_REMOTE) ;
    
   HAL_FDCAN_Start(&hfdcan2);
   HAL_FDCAN_ActivateNotification(&hfdcan2, 
                                      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 
                                      0) ;
}

/**
 * @brief FDCAN 接收回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }
    
    switch (rx_header.Identifier)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            uint8_t i = rx_header.Identifier - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief 发送底盘控制命令
 */
/*void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    FDCAN_TxHeaderTypeDef tx_header = {
        .Identifier = CAN_CHASSIS_ALL_ID,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0x00
    };
    
    uint8_t tx_data[8] = {
        (uint8_t)(motor1 >> 8),
        (uint8_t)(motor1),
        (uint8_t)(motor2 >> 8),
        (uint8_t)(motor2),
        (uint8_t)(motor3 >> 8),
        (uint8_t)(motor3),
        (uint8_t)(motor4 >> 8),
        (uint8_t)(motor4)
    };
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}*/
void FDCAN_cmd_motor_speed(uint8_t motor_id, float v_des)
{
    FDCAN_TxHeaderTypeDef tx_header = {
        .Identifier = 0x200 + motor_id,      // 帧ID = 0x200 + 电机ID (原CAN_CHASSIS_ALL_ID替换为动态ID)
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_4,     // 数据长度改为4字节（原为8字节）
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0x00
    };
    
    // 将浮点数转为4字节数组（小端序存储）
    union {
        float float_value;
        uint8_t bytes[4];
    } converter;
    
    converter.float_value = v_des;  // 将浮点数存储到联合体
    
    uint8_t tx_data[4] = {
        converter.bytes[0],  // D[0]：浮点数的低字节
        converter.bytes[1],  // D[1]
        converter.bytes[2],  // D[2]
        converter.bytes[3]   // D[3]：浮点数的高字节
    };
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}
void J4310_Enable()
{    
    Fdcan_Send_Data_t Fdcan_Send_Data;
    
    // 配置CAN报文头
    Fdcan_Send_Data.FDCAN_TxHeader.Identifier = 0x001;        // CAN ID
    Fdcan_Send_Data.FDCAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    Fdcan_Send_Data.FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    Fdcan_Send_Data.FDCAN_TxHeader.DataLength = 8;             // 数据长度8字节
    Fdcan_Send_Data.FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    Fdcan_Send_Data.FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    Fdcan_Send_Data.FDCAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    Fdcan_Send_Data.FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    Fdcan_Send_Data.FDCAN_TxHeader.MessageMarker = 0;
    
    // 设置使能指令
    Fdcan_Send_Data.FDCANx_Send_RxMessage[0] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[1] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[2] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[3] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[4] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[5] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[6] = 0xFF;
    Fdcan_Send_Data.FDCANx_Send_RxMessage[7] = 0xFC;  // 使能标志位
    
    // 发送FDCAN消息
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, 
                                  &Fdcan_Send_Data.FDCAN_TxHeader,
                                  Fdcan_Send_Data.FDCANx_Send_RxMessage);
}
/**
 * @brief 获取电机测量点
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i & 0x03];
}