/**
 * @file PID.h
 * @author Miraggio (w1159904119@gmail.com)
 * @brief PID控制器模块头文件
 * @version 0.1
 * @date 2021-03-30
 * @copyright Copyright (c) 2021
 */

#ifndef ___PID_H
#define ___PID_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kalman_filter.h"
#include "math.h"
#include "FuzzyPID.h"

/**********PID控制器数据结构接口************/

/**
 * @brief PID控制方式枚举
 */
typedef enum
{
    pid_control_increase,   ///< 增量式PID控制
    pid_control_normal,      ///< 标准位置式PID控制
    pid_control_frontfeed,   ///< 带前馈的位置式PID控制
    pid_control_frontfuzzy  ///< 带模糊控制的位置式PID控制
} pid_control;

/**
 * @brief 增量式PID控制器结构体
 */
typedef struct incrementalpid_t
{
    float Target;           ///< 设定目标值
    float Measured;         ///< 测量值
    float err;              ///< 当前偏差值
    float err_last;         ///< 上一次偏差
    float err_beforeLast;   ///< 上上次偏差
    float Kp;               ///< 比例系数
    float Ki;               ///< 积分系数
    float Kd;               ///< 微分系数
    float p_out;            ///< 比例项输出
    float i_out;            ///< 积分项输出
    float d_out;            ///< 微分项输出
    float pwm;              ///< PWM输出值
    uint32_t MaxOutput;     ///< 输出限幅值
    uint32_t IntegralLimit; ///< 积分项限幅值
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured); ///< 增量式PID计算函数指针
} incrementalpid_t;

/**
 * @brief 位置式PID控制器结构体
 */
typedef struct positionpid_t
{
    float Target;           ///< 设定目标值
    float Measured;         ///< 测量值
    float err;              ///< 当前偏差值
    float err_last;         ///< 上一次偏差
    float err_change;       ///< 偏差变化量
    float error_target;     ///< 目标值变化量（用于前馈）
    float last_set_point;   ///< 上一次目标值
    float Kp;               ///< 比例系数
    float Ki;               ///< 积分系数
    float Kd;               ///< 微分系数
    float Kf;               ///< 前馈系数
    float p_out;            ///< 比例项输出
    float i_out;            ///< 积分项输出
    float d_out;            ///< 微分项输出
    float f_out;            ///< 前馈项输出
    float pwm;              ///< PWM输出值
    float MaxOutput;        ///< 输出限幅值
    float Integral_Separation; ///< 积分分离阈值
    float IntegralLimit;    ///< 积分项限幅值
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured); ///< 位置式PID计算函数指针
} positionpid_t;

/**********外部变量声明************/

/// Yaw轴位置环PID控制器
extern positionpid_t M6020s_YawIPID;

/// Yaw轴速度环PID控制器
extern positionpid_t M6020s_Yaw_SpeedPID;

/// Yaw轴输出环PID控制器
extern positionpid_t M6020s_YawOPID;

/// 自动瞄准Yaw轴位置环PID控制器
extern positionpid_t AutoAim_M6020s_YawIPID;

/// 自动瞄准Yaw轴输出环PID控制器
extern positionpid_t AutoAim_M6020s_YawOPID;

/**********函数声明************/

/**
 * @brief 增量式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 */
extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

/**
 * @brief 位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 */
extern float Position_PID(positionpid_t *pid_t, float target, float measured);

/**
 * @brief 云台旋转PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 */
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);

/**
 * @brief 角度PID控制器（自动处理角度环绕）
 * @param pid_t PID控制器结构体指针
 * @param target 目标角度值
 * @param measured 测量角度值
 * @param ecd_max 被控电机编码器最大值（例如8192对应360度）
 * @return PID控制输出值
 */
extern float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max);

/**
 * @brief 底盘跟随模式控制函数
 * @param angle 目标角度
 * @param start_flag 启动标志
 */
extern void chassis_follow_mode(float angle, uint8_t start_flag);

/**
 * @brief 初始化增量式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param Kp 比例系数
 * @param Kd 微分系数
 * @param Ki 积分系数
 * @param MaxOutput 最大输出限制
 * @param IntegralLimit 积分项限幅值
 */
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);

/**
 * @brief 初始化位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param Kp 比例系数
 * @param Kd 微分系数
 * @param Ki 积分系数
 * @param Kf 前馈系数
 * @param MaxOutput 最大输出限制
 * @param IntegralLimit 积分项限幅值
 * @param Integral_Separation 积分分离阈值
 */
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki, float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);
extern float speed_angle_limit_pid(positionpid_t *pid_t, float speed_target, float speed_measured,float angle_measured);

/**********卡尔曼滤波器实例声明************/

/// Yaw轴卡尔曼滤波器实例
extern One_Kalman_t Cloud_YAWODKalman;

/// Pitch轴卡尔曼滤波器实例
extern One_Kalman_t Cloud_PITCHODKalman;

/**
 * @brief 清除位置式PID控制器数据
 * @param pid_t PID控制器结构体指针
 */
extern void Clear_PositionPIDData(positionpid_t *pid_t);

/**
 * @brief 清除增量式PID控制器数据
 * @param pid_t PID控制器结构体指针
 */
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

/**
 * @brief Yaw轴模糊位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param fuzzy_t 模糊PID数据结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 */
extern float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

#endif /* ___PID_H */