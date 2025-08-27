/**
 * @file N100.c
 * @author lxr(784457420@qq.com)
 * @brief N100 IMU传感器数据处理模块
 * @version 1.0
 * @date 2023-11-15
 * @copyright Copyright (c) 2023
 */

#include "N100.h"  // 包含N100传感器头文件

uint8_t N100_Rxbuffer[56];  // 定义接收缓冲区，用于存储从UART接收的原始数据
N100_AHRSData_Packet_t N100_Angle;  // 定义结构体变量，用于存储解析后的姿态数据

/*************
 * 功能：实现16进制数据转换为浮点数
 * 参数：四个字节的数据
 * 返回值：转换后的浮点数
 ****************/
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint32_t transition_32;  // 用于存储合并后的32位整数
    float tmp = 0;           // 存储转换后的浮点数
    int sign = 0;            // 符号位（0为正，1为负）
    int exponent = 0;        // 指数部分
    float mantissa = 0;      // 尾数部分

    // 将四个字节合并为一个32位整数
    transition_32 = 0;
    transition_32 |= Data_4 << 24;   // 最高有效字节
    transition_32 |= Data_3 << 16;   // 次高有效字节
    transition_32 |= Data_2 << 8;     // 次低有效字节
    transition_32 |= Data_1;          // 最低有效字节

    // 提取符号位（最高位）
    sign = (transition_32 & 0x80000000) ? -1 : 1;
    
    // 提取指数部分（30-23位）
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    
    // 提取尾数部分（22-0位）并转换为浮点数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    
    // 计算最终浮点数值：符号 × 尾数 × 2^指数
    tmp = sign * mantissa * pow(2, exponent);
    
    return tmp;  // 返回转换后的浮点数
}

/**
 * @brief 将四个字节合并为32位时间戳
 * @param Data_1 字节1（最低有效字节）
 * @param Data_2 字节2
 * @param Data_3 字节3
 * @param Data_4 字节4（最高有效字节）
 * @return 合并后的32位时间戳
 */
long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
    uint32_t transition_32;  // 用于存储合并后的32位整数
    
    // 将四个字节合并为一个32位整数
    transition_32 = 0;
    transition_32 |= Data_4 << 24;   // 最高有效字节
    transition_32 |= Data_3 << 16;   // 次高有效字节
    transition_32 |= Data_2 << 8;     // 次低有效字节
    transition_32 |= Data_1;          // 最低有效字节
    
    return transition_32;  // 返回合并后的时间戳
}

/**
 * @brief N100 IMU初始化函数，开启DMA接收
 * @retval none
 */
void N100_Init(void)
{
    // 使用DMA接收UART数据到缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, N100_Rxbuffer, sizeof(N100_Rxbuffer));
}

/**
 * @brief N100 IMU数据解析函数，将原始数据解析到结构体
 * @retval none
 */
void N100_Read(void)
{
    // 检查数据包类型和长度是否匹配AHRS数据
    if (N100_Rxbuffer[1] == TYPE_AHRS && N100_Rxbuffer[2] == AHRS_LEN)
    {
        // 解析角速度数据（滚转、俯仰、偏航）
        N100_Angle.RollSpeed = DATA_Trans(N100_Rxbuffer[7], N100_Rxbuffer[8], N100_Rxbuffer[9], N100_Rxbuffer[10]);       // 滚转角速度
        N100_Angle.PitchSpeed = DATA_Trans(N100_Rxbuffer[11], N100_Rxbuffer[12], N100_Rxbuffer[13], N100_Rxbuffer[14]); // 俯仰角速度
        N100_Angle.YawSpeed = DATA_Trans(N100_Rxbuffer[15], N100_Rxbuffer[16], N100_Rxbuffer[17], N100_Rxbuffer[18]);   // 偏航角速度
        
        // 解析姿态角数据（滚转、俯仰、偏航）
        N100_Angle.Roll = DATA_Trans(N100_Rxbuffer[19], N100_Rxbuffer[20], N100_Rxbuffer[21], N100_Rxbuffer[22]);       // 滚转角
        N100_Angle.Pitch = DATA_Trans(N100_Rxbuffer[23], N100_Rxbuffer[24], N100_Rxbuffer[25], N100_Rxbuffer[26]);       // 俯仰角
        N100_Angle.Yaw = DATA_Trans(N100_Rxbuffer[27], N100_Rxbuffer[28], N100_Rxbuffer[29], N100_Rxbuffer[30]);         // 偏航角
        
        // 解析四元数数据（姿态的四元数表示）
        N100_Angle.Qw = DATA_Trans(N100_Rxbuffer[31], N100_Rxbuffer[32], N100_Rxbuffer[33], N100_Rxbuffer[34]);          // 四元数w分量
        N100_Angle.Qx = DATA_Trans(N100_Rxbuffer[35], N100_Rxbuffer[36], N100_Rxbuffer[37], N100_Rxbuffer[38]);         // 四元数x分量
        N100_Angle.Qy = DATA_Trans(N100_Rxbuffer[39], N100_Rxbuffer[40], N100_Rxbuffer[41], N100_Rxbuffer[42]);         // 四元数y分量
        N100_Angle.Qz = DATA_Trans(N100_Rxbuffer[43], N100_Rxbuffer[44], N100_Rxbuffer[45], N100_Rxbuffer[46]);         // 四元数z分量
        
        // 解析时间戳数据
        N100_Angle.Timestamp = timestamp(N100_Rxbuffer[47], N100_Rxbuffer[48], N100_Rxbuffer[49], N100_Rxbuffer[50]);    // 时间戳
    }
}