/* 版权声明和文件描述 */
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      PID实现函数，包括初始化函数和PID计算函数
  * @note       注意点说明
  * @history    版本历史
  *  Version    Date            Author          centre
  *  V1.0.0     Aug-11-2025     RM              1. 完成初始版本
  */

#ifndef PID_H  // 防止头文件重复包含
#define PID_H

#include "struct_typedef.h"  // 包含自定义类型定义的头文件

// PID控制模式枚举
enum PID_MODE
{
    PID_POSITION = 0,  // 位置式PID
    PID_DELTA          // 增量式PID
};

// PID控制器结构体定义
typedef struct
{
    uint8_t mode;       // PID控制模式（位置式/增量式）
    
    // PID参数
    fp32 Kp;           // 比例系数
    fp32 Ki;           // 积分系数
    fp32 Kd;           // 微分系数
    fp32 Kf;           // 前馈系数

    fp32 max_out;       // PID输出最大值限制
    fp32 max_iout;      // 积分项最大值限制（抗积分饱和）

    fp32 target;           // 设定值（目标值）
    fp32 measured;           // 反馈值（当前值）
    fp32 last_target;       // 上一次设定值（用于增量式PID）

    fp32 out;           // PID输出值
    fp32 Pout;          // 比例项输出
    fp32 Iout;          // 积分项输出
    fp32 Dout;          // 微分项输出
    fp32 Fout;          // 前馈项输出（如果有的话）
    fp32 Dbuf[3];       // 微分项缓冲区（0:当前值, 1:上一次值, 2:上上次值）
    fp32 error[3];      // 误差缓冲区（0:当前误差, 1:上一次误差, 2:上上次误差）

} pid_type_def;

/**
  * @brief          PID结构体数据初始化
  * @param[out]     pid: PID结构体指针
  * @param[in]      mode: PID_POSITION: 普通位置式PID
  *                      PID_DELTA: 增量式PID
  * @param[in]      PID: 数组指针 [0]: Kp, [1]: Ki, [2]: Kd,[3]: Kf
  * @param[in]      max_out: PID输出最大值
  * @param[in]      max_iout: PID积分项最大值
  * @retval         无
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[4], fp32 max_out, fp32 max_iout);

/**
  * @brief          PID计算函数
  * @param[out]     pid: PID结构体指针
  * @param[in]      ref: 反馈值（当前值）
  * @param[in]      set: 设定值（目标值）
  * @retval         PID输出值
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          清除PID输出（重置PID状态）
  * @param[out]     pid: PID结构体指针
  * @retval         无
  */
extern void PID_clear(pid_type_def *pid);

#endif  // 结束头文件