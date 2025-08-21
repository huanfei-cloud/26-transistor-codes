/* 版权声明和文件描述 */
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      PID实现函数，包括初始化函数和PID计算函数
  * @note       注意点说明
  * @history    版本历史
  *  Version    Date            Author          centre
  *  V1.0.0     Aug-11-2025     RM              1. 完成初始版本
  */

#include "pid.h"    // PID控制头文件
#include "main.h"   // 主工程头文件

// 定义限幅宏：限制输入值在[-max, max]范围内
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          PID结构体数据初始化
  * @param[out]     pid: PID结构体指针
  * @param[in]      mode: PID_POSITION: 普通位置式PID
  *                      PID_DELTA: 增量式PID
  * @param[in]      PID: 数组指针 [0]: Kp, [1]: Ki, [2]:kd, [3]: Kf
  * @param[in]      max_out: PID输出最大值
  * @param[in]      max_iout: PID积分项最大值
  * @retval         无
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[4], fp32 max_out, fp32 max_iout)
{
    // 检查输入指针是否有效
    if (pid == NULL || PID == NULL)
    {
        return;  // 无效指针直接返回
    }
    
    // 设置PID模式
    pid->mode = mode;
    
    // 设置PID参数
    pid->Kp = PID[0];  // 比例系数
    pid->Ki = PID[1];  // 积分系数
    pid->Kd = PID[2];  // 微分系数
    pid->Kf = PID[3]; // 前馈系数（如果有的话）

    // 设置输出限制
    pid->max_out = max_out;   // PID总输出最大值
    pid->max_iout = max_iout; // 积分项最大值
    
    // 初始化缓冲区
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;    // 微分项缓冲区清零
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f; // 误差缓冲区清零
    
    // 初始化输出项
    pid->Pout = 0.0f;  // 比例项输出
    pid->Iout = 0.0f;  // 积分项输出
    pid->Dout = 0.0f;  // 微分项输出
    pid->Fout = 0.0f;  // 前馈项输出（如果有的话）
    pid->out = 0.0f;   // PID总输出
    pid->target = 0.0f;     // 设定值（目标值）
    pid->last_target = 0.0f; // 上一次设定值（用于增量式PID）
}

/**
  * @brief          PID计算函数
  * @param[out]     pid: PID结构体指针
  * @param[in]      measured: 反馈值（当前值）
  * @param[in]      target: 设定值（目标值）
  * @retval         PID输出值
  */
fp32 PID_calc(pid_type_def *pid, fp32 measured, fp32 target)
{
    // 检查输入指针是否有效
    if (pid == NULL)
    {
        return 0.0f;  // 无效指针返回0
    }
    
    // 更新历史误差：将前一次误差移动到更早的位置
    pid->error[2] = pid->error[1];  // 将上一次误差移动到上上次位置
    pid->error[1] = pid->error[0];  // 将当前误差移动到上一次位置
    pid->last_target = pid->target;  // 保存上一次设定值
    // 更新设定值和反馈值
    pid->target = target;  // 设置目标值
    pid->measured = measured;  // 设置当前值
    
    // 计算当前误差：目标值 - 当前值
    pid->error[0] = target - measured;
    
    // 根据PID模式进行计算
    if (pid->mode == PID_POSITION)  // 位置式PID
    {
        // 比例项计算
        pid->Pout = pid->Kp * pid->error[0];
        
        // 积分项计算（累加）
        pid->Iout += pid->Ki * pid->error[0];
        
        // 更新微分项历史值
        pid->Dbuf[2] = pid->Dbuf[1];  // 将上一次微分值移动到上上次位置
        pid->Dbuf[1] = pid->Dbuf[0];  // 将当前微分值移动到上一次位置
        
        // 计算当前微分值：当前误差 - 上一次误差
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        
        // 微分项计算
        pid->Dout = pid->Kd * pid->Dbuf[0];

        // 前馈项计算（如果有的话）
        pid->Fout = pid->Kf * (pid->target-pid->last_target);  // 前馈
        
        // 积分项限幅（抗积分饱和）
        LimitMax(pid->Iout, pid->max_iout);
        
        // 计算PID总输出：比例 + 积分 + 微分
        pid->out = pid->Pout + pid->Iout + pid->Dout+pid->Fout;
        
        // PID总输出限幅
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)  // 增量式PID
    {
        // 比例项计算：Kp * (当前误差 - 上一次误差)
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        
        // 积分项计算：Ki * 当前误差
        pid->Iout = pid->Ki * pid->error[0];
        
        // 更新微分项历史值
        pid->Dbuf[2] = pid->Dbuf[1];  // 将上一次微分值移动到上上次位置
        pid->Dbuf[1] = pid->Dbuf[0];  // 将当前微分值移动到上一次位置
        
        // 计算二阶微分：当前误差 - 2*上一次误差 + 上上次误差
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        
        // 微分项计算
        pid->Dout = pid->Kd * pid->Dbuf[0];
       
        // 计算PID增量输出：比例 + 积分 + 微分
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        
        // PID总输出限幅
        LimitMax(pid->out, pid->max_out);
    }
    
    // 返回PID输出值
    return pid->out;
}

/**
  * @brief          清除PID输出（重置PID状态）
  * @param[out]     pid: PID结构体指针
  * @retval         无
  */
void PID_clear(pid_type_def *pid)
{
    // 检查输入指针是否有效
    if (pid == NULL)
    {
        return;  // 无效指针直接返回
    }
    
    // 清零误差缓冲区
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    
    // 清零微分项缓冲区
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    
    // 清零所有输出项
    pid->out = 0.0f;   // PID总输出
    pid->Pout = 0.0f;  // 比例项输出
    pid->Iout = 0.0f;  // 积分项输出
    pid->Dout = 0.0f;  // 微分项输出
    pid->Fout = 0.0f;  // 前馈项输出（如果有的话）
    
    // 清零设定值和反馈值
    pid->measured = 0.0f;  // 反馈值
    pid->target = 0.0f;  // 设定值
    pid->last_target = 0.0f; // 上一次设定值（用于增量式PID）
}