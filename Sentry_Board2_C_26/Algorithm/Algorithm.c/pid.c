/**
 * @file PID.c
 * @author centre
 * @brief PID控制算法实现模块
 * @version 0.2
 * @date 2025-08-21
 * @copyright Copyright (c) 2021
 */

#include "PID.h"

One_Kalman_t Cloud_YAWODKalman; ///< YAW轴卡尔曼滤波器实例
One_Kalman_t Cloud_PITCHODKalman; ///< PITCH轴卡尔曼滤波器实例

/**
 * @brief 绝对值限制函数
 * @param a 指向要限制的浮点数的指针
 * @param ABS_MAX 绝对值的最大限制
 * @note 此函数会直接修改传入的指针值
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/**
 * @brief Yaw轴模糊位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param fuzzy_t 模糊PID数据结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 * @note 此函数结合模糊控制优化PID参数，适用于Yaw轴控制
 */
float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured)
{
    // 模糊计算更新PID参数
    FuzzyComputation(fuzzy_t, pid_t->err, pid_t->err_last);
    
    // 更新状态变量
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;

    // 计算各项输出（使用模糊调整后的参数）
    pid_t->p_out = (pid_t->Kp + fuzzy_t->deta_kp) * pid_t->err;
    pid_t->i_out += (pid_t->Ki + fuzzy_t->date_ki) * pid_t->err;
    pid_t->d_out = (pid_t->Kd + fuzzy_t->date_kd) * (pid_t->Measured - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // 积分分离策略
    if (pid_t->err >= pid_t->Integral_Separation) {
        pid_t->i_out = 0;
    } else {
        // 积分限幅
        abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
    }

    // 计算总输出
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // 输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // 更新历史状态
    pid_t->err_last = pid_t->Measured;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}

/**
 * @brief 增量式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 * @note 适用于需要平滑控制的场景
 */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{
    // 更新状态变量
    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    // 计算各项输出
    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    // 积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // 计算总输出
    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    // 输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // 更新历史状态
    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}

/**
 * @brief 初始化增量式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param MaxOutput 最大输出限制
 * @param IntegralLimit 积分项限幅值
 */
void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit)
{
    // 设置PID参数
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    
    // 设置限制参数
    pid_t->MaxOutput = MaxOutput;
    pid_t->IntegralLimit = IntegralLimit;
    
    // 初始化状态变量
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

/**
 * @brief 清除增量式PID控制器数据
 * @param pid_t PID控制器结构体指针
 * @note 重置所有状态变量，保留参数设置
 */
void Clear_IncrementalPIDData(incrementalpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/**
 * @brief 位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID控制输出值
 * @note 标准位置式PID实现，带前馈控制
 */
float Position_PID(positionpid_t *pid_t, float target, float measured)
{
    // 更新状态变量
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // 计算各项输出
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // 积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // 计算总输出
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // 输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // 更新历史状态
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}

/**
 * @brief 初始化位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param Kf 前馈系数
 * @param MaxOutput 最大输出限制
 * @param Integral_Separation 积分分离阈值
 * @param IntegralLimit 积分项限幅值
 */
void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float Kf, float MaxOutput, float Integral_Separation, float IntegralLimit)
{
    // 设置PID参数
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->Kf = Kf;
    
    // 设置限制参数
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
    
    // 初始化状态变量
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_change = 0;
    pid_t->error_target = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

/**
 * @brief 清除位置式PID控制器数据
 * @param pid_t PID控制器结构体指针
 * @note 重置所有状态变量，保留参数设置
 */
void Clear_PositionPIDData(positionpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_change = 0;
    pid_t->err_last = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/**
 * @brief 角度位置式PID控制器
 * @param pid_t PID控制器结构体指针
 * @param target 目标角度值
 * @param measured 测量角度值
 * @param ecd_max 被控电机编码器最大值（例如8191对应360度）
 * @return PID控制输出值
 * @note 专门处理电机角度值的PID控制器，自动处理角度环绕问题，编码值需要从0开始
 */
float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max)
{
    // 更新状态变量
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    
    // 处理角度环绕问题（8192对应360度）
    if(abs(pid_t->err) > ecd_max/2) {
        if(pid_t->err > 0) {
            pid_t->err = pid_t->err - (ecd_max+1);
        } else {
            pid_t->err = pid_t->err + (ecd_max+1);
        }
    }
    
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // 计算各项输出
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    
    // 积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // 计算总输出
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // 输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // 更新历史状态
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;
}
/**
 * @brief 特定位置降kp kd的PID控制器
 * @param pid_t PID控制器结构体指针
 * @param speed_target 目标速度值
 * @param speed_measured 测量速度值
 * @param angle_measured 角度测量值
 * @return PID控制输出值
 * @note 专门处理6020电机特定角度值抖动的速度环pid函数
 */
float speed_angle_limit_pid(positionpid_t *pid_t, float speed_target, float speed_measured,float angle_measured)
{
// 更新状态变量
    pid_t->Target = (float)speed_target;
    pid_t->Measured = (float)speed_measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
    
    // 计算各项输出
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->f_out = pid_t->Kf * pid_t->error_target;
    if((angle_measured>5700&&angle_measured<8000)||(angle_measured>1000&&angle_measured<3300))
    {
        pid_t->p_out = 40 * pid_t->err;
        pid_t->d_out = 20 * (pid_t->err - pid_t->err_last);
    }
    // 积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    // 计算总输出
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    // 输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    // 更新历史状态
    pid_t->err_last = pid_t->err;
    pid_t->last_set_point = pid_t->Target;
    
    return pid_t->pwm;

}
