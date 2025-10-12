/**
 * @file M6020_Motor.c
 * @author Miraggio (w1159904119@gmail.com)
 * @brief M6020电机控制实现
 * @version 0.1
 * @date 2021-03-30
 * @copyright Copyright (c) 2021
 */

#include "M6020_Motor.h"  // 包含电机头文件

/******************** 全局变量定义 ********************/
// 直接声明电机结构体（便于调试观察）
M6020s_t M6020s_Yaw;                                    // ID1电机实例
M6020s_t *M6020_Array[1] = {&M6020s_Yaw};               // 电机指针数组（索引0对应ID1）

#define M6020_Amount 1  // 电机数量

/********函数声明********/
void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
void M6020_getInfo(Can_Export_Data_t RxMessage);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);

// 初始化电机功能函数结构体
M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit  // 取消初始化宏定义

/**
 * @brief  设置M6020电机控制值（打包为CAN数据）
 * @param  uq1~uq4: 四个电机的控制值（-30000~30000）
 * @param  data: CAN数据缓冲区（8字节）
 * @retval None
 * @note 控制值范围由M6020_MaxOutput定义
 */
void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data)
{
    // 将16位控制值拆分为高低字节存入缓冲区
    data[0] = uq1 >> 8;  // uq1高字节
    data[1] = uq1;       // uq1低字节
    data[2] = uq2 >> 8;  // uq2高字节
    data[3] = uq2;       // uq2低字节
    data[4] = uq3 >> 8;  // uq3高字节
    data[5] = uq3;       // uq3低字节
    data[6] = uq4 >> 8;  // uq4高字节
    data[7] = uq4;       // uq4低字节
}

/**
 * @brief  从CAN报文解析电机状态信息
 * @param  RxMessage: CAN接收数据结构体
 * @retval None
 * @note 数据格式详见C620电调说明书P33
 */
void M6020_getInfo(Can_Export_Data_t RxMessage)
{
    // 计算电机索引（0开始）
    int32_t StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START;
    
    // 边界检查（此处被注释，实际使用时应启用）
    // if (IndexOutofBounds(StdId, M6020_Amount)) {
    //     Device_setAlertType(Alert_Times_SoftWare);
    //     return;
    // }
    
    // 保存上次角度（用于圈数计算）
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    
    // 解析CAN数据（大端序）
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | 
                                              RxMessage.CANx_Export_RxMessage[1]);
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | 
                                              RxMessage.CANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | 
                                                RxMessage.CANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.CANx_Export_RxMessage[6];
    
    /******************** 多圈计数处理 ********************/
    // 检测逆时针跨越零点（从8191跳转到0附近）
    if (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -6500) {
        M6020_Array[StdId]->turnCount++;  // 增加圈数
    }
    // 检测顺时针跨越零点（从0跳转到8191附近）
    if (M6020_Array[StdId]->lastAngle - M6020_Array[StdId]->realAngle < -6500) {
        M6020_Array[StdId]->turnCount--;  // 减少圈数
    }
    
    // 计算总角度：当前角度 + 圈数*8192
    M6020_Array[StdId]->totalAngle = M6020_Array[StdId]->realAngle + 
                                    (8192 * M6020_Array[StdId]->turnCount);
    
    // 更新状态标志
    M6020_Array[StdId]->InfoUpdateFrame++;  // 更新帧计数
    M6020_Array[StdId]->InfoUpdateFlag = 1;  // 设置更新标志
}

/**
 * @brief  设置电机目标角度
 * @param  M6020: 电机结构体指针
 * @param  angle: 目标机械角度（0-8191）
 * @retval None
 * @note 实际使用时需配合PID控制器
 */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    // 简单赋值目标角度（外部需处理角度归一化）
    M6020->targetAngle = angle;
}

/**
 * @brief  重置电机多圈计数和总角度
 * @param  m6020: 电机结构体指针
 * @retval None
 * @note 解决长时间运行导致的totalAngle溢出问题
 */
void M6020_Reset(M6020s_t *m6020)
{
    m6020->lastAngle = m6020->realAngle;  // 更新上次角度
    m6020->totalAngle = m6020->realAngle; // 重置总角度（单圈）
    m6020->turnCount = 0;                 // 清零圈数计数
}