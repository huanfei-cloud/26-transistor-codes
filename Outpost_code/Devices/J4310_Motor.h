/**
 * @file D4310_Motor.h
 * @author ZS
 * @brief J4310电机驱动头文件（FDCAN版本）
 * @version 0.2
 * @date 2024-12-20
 * 
 * @copyright Copyright (c)
 *
 */
#ifndef __J4310_MOTOR_H
#define __J4310_MOTOR_H

#include "main.h"
#include "typedef.h"
#include "PID.h"
#include "BSP_Fdcan.h"
#include "fdcan.h"
// 电机通信参数
#define J4310_READID_PITCH 0x01      // 电机接收ID
#define J4310_SENDID_PITCH 0x001     // 电机发送ID
#define J4310_MaxV 200               // 最大转速 (rpm)
#define J4310_MaxT 7                 // 最大扭矩 (Nm)
#define J4310_ReductionRatio 10      // 减速比

// 电机初始位置和角度转换参数
#define J4310_FIRSTANGLE 3800        // 电机初始位置
#define J4310_mAngleRatio 22.7527f   // 机械角度与真实角度的比率
#define Pi 3.14159265f               // 圆周率

// 函数指针宏定义
#define J4310_FunGroundInit        \
    {                              \
        &J4310_setParameter,       \
        &J4310_Enable,             \
        &J4310_Save_Pos_Zero,      \
        &J4310_getInfo,            \
        &J4310_setTargetAngle,     \
        &J4310_Reset,              \
        &Check_J4310,              \
    }

/**
 * @brief J4310电机状态结构体
 */
typedef struct
{
    int16_t  state;                // 电机状态
    float realAngle;                // 实际机械角度（度）
    float realSpeed;                // 实际转速（rpm）
    uint8_t temperatureMOS;        // MOS温度
    uint8_t temperatureRotor;     // 转子温度
    float torqueInit;               // 原始扭矩值
    float torque;                   // 计算后的扭矩（Nm）
    float angleInit;                // 原始角度值
    float speedInit;                // 原始速度值
    
    uint16_t lastAngle;            // 上次的角度值
    int32_t targetSpeed;            // 目标转速
    int32_t targetAngle;            // 目标角度
    
    float outPosition;              // 输出位置
    float outSpeed;                 // 输出速度
    float outTorque;                // 输出扭矩
    
    int16_t turnCount;              // 转过的圈数
    float totalAngle;               // 累积总角度

    int8_t outKp;                   // 位置环比例系数
    int8_t outKd;                   // 位置环微分系数

    uint8_t InfoUpdateFlag;          // 信息更新标志
    uint16_t InfoUpdateFrame;        // 信息更新帧计数
    uint8_t OffLineFlag;             // 离线标志
} J4310s_t;

/**
 * @brief 电机名称枚举
 */
typedef enum
{
    J4310_PITCH = 0,    // Pitch轴电机
} J4310Name_e;

/**
 * @brief FDCAN接收数据结构体
 */
typedef struct
{
    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;   // FDCAN接收头
    uint8_t FDCANx_Export_RxMessage[8];    // 接收数据
} FDCAN_Export_Data_t;

/**
 * @brief J4310电机功能函数结构体
 */
typedef struct
{
    void (*J4310_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
    void (*J4310_Enable)(void);
    void (*J4310_Save_Pos_Zero)(void);
    void (*J4310_getInfo)(FDCAN_Export_Data_t RxMessage);
    void (*J4310_setTargetAngle)(J4310s_t *J4310, int32_t angle);
    void (*J4310_Reset)(J4310s_t *J4310);
    void (*Check_J4310)(void);
} J4310_Fun_t;

// 外部声明
extern J4310s_t J4310s_Pitch;   // Pitch轴电机实例
extern J4310_Fun_t J4310_Fun;    // 电机功能函数集

#endif /* __J4310_MOTOR_H */