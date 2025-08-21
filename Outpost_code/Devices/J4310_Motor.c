/**
 * @file J4310_Motor.c
 * @author ZS (2729511164@qq.com)
 * @brief J4310电机驱动实现（FDCAN版本）
 * @version 0.2
 * @date 2024-12-20
 * 
 * @copyright Copyright (c)
 *
 */
#include "J4310_Motor.h"
#include "BSP_Fdcan.h"  // FDCAN支持
#include "main.h"

// 电机实例
J4310s_t J4310s_Pitch;                      
J4310s_t *J4310_Array[1] = {&J4310s_Pitch}; // 电机指针数组

#define J4310_Amount 1  // 电机数量

// 函数声明
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
static int float_to_uint(float X_float, float X_min, float X_max, int bits);
void J4310_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
void J4310_Enable(void);
void J4310_Save_Pos_Zero(void);
void J4310_getInfo(FDCAN_Export_Data_t RxMessage);
void J4310_setTargetAngle(J4310s_t *J4310, int32_t angle);
void J4310_Reset(J4310s_t *J4310);
void Check_J4310(void);

// 初始化功能函数集
J4310_Fun_t J4310_Fun = J4310_FunGroundInit;
#undef J4310_FunGroundInit

/**
 * @brief 将整型数据转换为浮点数
 * @param X_int 整型值
 * @param X_min 最小值
 * @param X_max 最大值
 * @param Bits 位数
 * @return 转换后的浮点数
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

/**
 * @brief 将浮点数转换为整型数据
 * @param X_float 浮点值
 * @param X_min 最小值
 * @param X_max 最大值
 * @param bits 位数
 * @return 转换后的整型数
 */
static int float_to_uint(float X_float, float X_min, float X_max, int bits)
{
    float span = X_max - X_min;
    float offset = X_min;
    return (int)((X_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 设置J4310电机控制参数
 * @param uq1 位置
 * @param uq2 速度
 * @param uq3 Kp
 * @param uq4 Kd
 * @param uq5 扭矩
 * @param data 输出数据缓冲区
 */
void J4310_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data)
{
    // 位置归一化处理
    float position = uq1 / 8192.0f * 2.0f * Pi;
    
    // 定义参数范围
    const float P_MAX = 3.141593f;  // 位置最大值 (rad)
    const float V_MAX = 200.0f;      // 速度最大值 (rpm)
    const float T_MAX = 7.0f;        // 扭矩最大值 (Nm)
    
    // 转换参数为整型
    uint16_t Postion_Tmp = float_to_uint(position, -P_MAX, P_MAX, 16);
    uint16_t Velocity_Tmp = float_to_uint(uq2, -V_MAX, V_MAX, 12);
    uint16_t Torque_Tmp = float_to_uint(uq5, -T_MAX, T_MAX, 12);
    uint16_t KP_Tmp = float_to_uint(uq3, 0, 500, 12);
    uint16_t KD_Tmp = float_to_uint(uq4, 0, 5, 12);
    
    // 填充数据缓冲区
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
 * @brief 使能J4310电机
 */
void J4310_Enable()
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[8] = {0};
    
    // 配置FDCAN发送头
    TxHeader.Identifier = J4310_SENDID_PITCH;    // 目标ID
    TxHeader.IdType = FDCAN_STANDARD_ID;          // 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;      // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;       // 8字节数据
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 关闭比特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 经典CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    // 填充使能命令数据
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;  // 使能命令
    
    // 发送FDCAN消息
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
        // 错误处理
        Error_Handler();
    }
}

/**
 * @brief 保存当前位置为零点
 */
void J4310_Save_Pos_Zero(void)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t data[8] = {0};
    
    // 配置FDCAN发送头
    TxHeader.Identifier = J4310_SENDID_PITCH;    // 目标ID
    TxHeader.IdType = FDCAN_STANDARD_ID;          // 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;      // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;       // 8字节数据
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 关闭比特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 经典CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    // 填充保存零点命令数据
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;  // 保存零点命令
    
    // 发送FDCAN消息
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
        // 错误处理
        Error_Handler();
    }
}

/**
 * @brief 从FDCAN报文解析电机信息
 * @param RxMessage FDCAN接收数据结构
 */
void J4310_getInfo(FDCAN_Export_Data_t RxMessage)
{
    // 计算电机索引
    int32_t StdId = RxMessage.FDCAN_RxHeader.Identifier - J4310_READID_PITCH;
    
    // 参数范围定义
    const float P_MAX = 12.5f;   // 位置最大值
    const float V_MAX = 200.0f;   // 速度最大值
    const float T_MAX = 7.0f;     // 扭矩最大值
    
    // 保存上一次角度
    J4310_Array[StdId]->lastAngle = J4310_Array[StdId]->realAngle;
    
    // 解析原始数据
    uint8_t* rxData = RxMessage.FDCANx_Export_RxMessage;
    J4310_Array[StdId]->state = rxData[0] >> 4;
    J4310_Array[StdId]->angleInit = (uint16_t)((rxData[1] << 8) | rxData[2]);
    J4310_Array[StdId]->speedInit = (uint16_t)((rxData[3] << 4) | (rxData[4] >> 4));
    J4310_Array[StdId]->torqueInit = (uint16_t)(((rxData[4] & 0x0F) << 8) | rxData[5]);
    
    // 转换原始数据为物理量
    J4310_Array[StdId]->realAngle = uint_to_float(J4310_Array[StdId]->angleInit, -P_MAX, P_MAX, 16);
    J4310_Array[StdId]->realAngle = J4310_Array[StdId]->realAngle / 2.0f * Pi * 36.0f;
    J4310_Array[StdId]->realSpeed = uint_to_float(J4310_Array[StdId]->speedInit, -V_MAX, V_MAX, 12);
    J4310_Array[StdId]->torque = uint_to_float(J4310_Array[StdId]->torqueInit, -T_MAX, T_MAX, 12);
    J4310_Array[StdId]->temperatureMOS = rxData[6];
    J4310_Array[StdId]->temperatureRotor = rxData[7];
    
    // 处理角度翻转
    if (J4310_Array[StdId]->realAngle - J4310_Array[StdId]->lastAngle < -6500.0f)
    {
        J4310_Array[StdId]->turnCount++;
    }
    else if (J4310_Array[StdId]->lastAngle - J4310_Array[StdId]->realAngle < -6500.0f)
    {
        J4310_Array[StdId]->turnCount--;
    }
    
    // 计算总角度
    J4310_Array[StdId]->totalAngle = J4310_Array[StdId]->realAngle + (8192.0f * J4310_Array[StdId]->turnCount);
    
    // 更新状态标志
    J4310_Array[StdId]->InfoUpdateFrame++;
    J4310_Array[StdId]->InfoUpdateFlag = 1;
}

/**
 * @brief 设置电机目标角度
 * @param J4310 电机实例指针
 * @param angle 目标角度（度）
 */
void J4310_setTargetAngle(J4310s_t *J4310, int32_t angle)
{
    J4310->targetAngle = angle;
}

/**
 * @brief 重置电机角度计数
 * @param J4310 电机实例指针
 */
void J4310_Reset(J4310s_t *J4310)
{
    J4310->lastAngle = J4310->realAngle;
    J4310->totalAngle = J4310->realAngle;
    J4310->turnCount = 0;
}

/**
 * @brief 检查电机在线状态
 */
void Check_J4310(void)
{
    for (int i = 0; i < J4310_Amount; i++)
    {
        // 检查帧更新计数判断是否离线
        if (J4310_Array[i]->InfoUpdateFrame < 1)
        {
            J4310_Array[i]->OffLineFlag = 1;  // 设置离线标志
        }
        else
        {
            J4310_Array[i]->OffLineFlag = 0;  // 清除离线标志
        }
        
        // 重置帧计数器
        J4310_Array[i]->InfoUpdateFrame = 0;
    }
}