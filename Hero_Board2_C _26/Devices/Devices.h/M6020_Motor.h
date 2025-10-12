/**
 * @file M6020_Motor.h
 * @author Miraggio (w1159904119@gmail.com)
 * @brief M6020电机控制头文件
 * @version 0.1
 * @date 2021-03-30
 * @copyright Copyright (c) 2021
 */
#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "can.h"          // CAN通信相关
#include "typedef.h"      // 自定义类型定义
#include <stdbool.h>      // 布尔类型支持
#include <stdint.h>       // 标准整数类型
#include <stdio.h>        // 标准输入输出
#include "pid.h"          // PID控制相关

/******************** 宏定义 ********************/
// CAN通信ID定义
#define M6020_READID_START 0x208 // 起始接收ID（ID1对应的接收ID）
#define M6020_READID_END 0x208   // 结束接收ID（ID4对应的接收ID）
#define M6020_YAW_ID 0x208       // 偏航轴电机接收ID
#define M6020_SENDID 0x1FF       // 发送ID（1-4号电机控制指令）

// 电机参数定义
#define M6020_CurrentRatio 0f    // 电流比率（待测定）
#define M6020_MaxOutput 30000    // 最大输出值（发送给电机的控制值范围）
#define M6020_mAngle 8191        // 电机机械角度最大值（0-8191对应360°）

// 角度转换参数
#define M6020_mAngleRatio 22.7527f // 机械角度与真实角度比率（8191/360）
// 将机械角度转换为真实角度（度）的宏
#define M6020_getRoundAngle(rAngle) rAngle / M6020_mAngleRatio

// 函数指针结构体初始化宏
#define M6020_FunGroundInit        \
    {                              \
        &M6020_setVoltage,         \
        &M6020_getInfo,            \
        &M6020_setTargetAngle,     \
        &M6020_Reset,              \
    }

/******************** 类型定义 ********************/
// M6020电机数据结构体
typedef struct
{
    // 电机反馈数据
    uint16_t realAngle;   // 当前机械角度（0-8191）
    int32_t realSpeed;    // 当前转速（RPM）
    int16_t realCurrent;  // 当前实际转矩电流
    uint8_t temperture;   // 电机温度
    
    // 状态记录
    uint16_t lastAngle;   // 上次角度（用于计算圈数）
    int16_t turnCount;    // 转过的圈数（多圈计数）
    float totalAngle;     // 累积总角度（度）
    
    // 控制目标
    float targetSpeed;    // 目标转速
    int32_t targetAngle;  // 目标角度（编码值）
    
    // 控制输出
    int16_t outCurrent;   // 输出电流值
    
    // 状态标志
    uint8_t InfoUpdateFlag;   // 信息更新标志（1=收到新数据）
    uint16_t InfoUpdateFrame;  // 信息更新帧计数
    uint8_t OffLineFlag;       // 离线标志（1=电机离线）
} M6020s_t;

// 电机名称枚举
typedef enum
{
    M6020_YAW = 0,  // 偏航轴电机（索引0）
} M6020Name_e;

// M6020功能函数结构体
typedef struct
{
    // 设置四个电机的电压（电流）值
    void (*M6020_setVoltage)(int16_t uq1, int16_t uq2, 
                             int16_t uq3, int16_t uq4, 
                             uint8_t *data);
    
    // 从CAN报文解析电机信息
    void (*M6020_getInfo)(Can_Export_Data_t RxMessage);
    
    // 设置目标角度（带多圈处理）
    void (*M6020_setTargetAngle)(M6020s_t *M6020, int32_t angle);
    
    // 重置电机数据（清零圈数和总角度）
    void (*M6020_Reset)(M6020s_t *m6020);
} M6020_Fun_t;

/******************** 全局变量声明 ********************/
extern M6020s_t M6020s_Yaw;   // 偏航轴电机实例（ID=1）
extern M6020_Fun_t M6020_Fun; // 电机功能函数实例

#endif /* __M6020_MOTOR_H */