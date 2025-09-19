/**
 * @file PID.h
 * @author Miraggio (w1159904119@gmail)
 * @brief
 * @version 0.1
 * @date 2021-03-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef ___PID_H
#define ___PID_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kalman_filter.h"
#include "FuzzyPID.h"
#include "math.h"

#define yaw_O_p 0.083f
#define yaw_O_i 0.00001f
#define yaw_O_d 0.0f

// #define yaw_I_p 1050.0f
#define yaw_I_p 10.0f
#define yaw_I_i 0.12f
#define yaw_I_d 10.0f
// #define yaw_I_d 1000.0f

#define yaw_O_Aim_p 2.1f
#define yaw_O_Aim_i 0.00001f
#define yaw_O_Aim_d 1.0f
#define yaw_O_Aim_f 0.0f

// #define yaw_I_Aim_p 1100.0f
#define yaw_I_Aim_p 170.0f
// #define yaw_I_Aim_i 60.0f
#define yaw_I_Aim_i 5.0f
// #define yaw_I_Aim_d 950.0f
#define yaw_I_Aim_d 50.0f
#define yaw_I_Aim_f 15.0f

/**********PID对外数据接口************/

// 增量式PID计算的基准参数
typedef struct incrementalpid_t
{
    float Target;         // 设定目标值
    float Measured;       // 测量值
    float err;            // 本次偏差值
    float err_last;       // 上一次偏差
    float err_beforeLast; // 上上次偏差
    float last_set_point; // 上一次目标值
    float error_target;   // 前馈控制
    float integral_error; // 误差积分累积
    float Kp;
    float Ki;
    float Kd;
    float kf; // Kp, Ki, Kd，Kf控制系数
    float p_out;
    float i_out;
    float d_out;            // 各部分输出值
    float f_out;            // 前馈f输出
    float pwm;              // pwm输出
    uint32_t MaxOutput;     // 输出限幅
    uint32_t IntegralLimit; // 积分限幅
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;

// 位置式PID计算的基准参数
typedef struct positionpid_t
{
    float Target;         // 设定目标值
    float Measured;       // 测量值
    float err;            // 本次偏差值
    float err_last;       // 上一次偏差
    float err_change;     // 误差变化率
    float last_set_point; // 上一次目标值
    float error_target;   // 前馈控制
    float integral_error; // 误差积分累积
    float Kp;
    float Ki;
    float Kd; // Kp, Ki, Kd控制系数
    float kf;
    float p_out;
    float i_out;
    float d_out;               // 各部分输出值
    float f_out;               // 前馈f输出
    float pwm;                 // pwm输出
    float MaxOutput;           // 输出限幅
    float Integral_Separation; // 积分分离阈值
    float IntegralLimit;       // 积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;

// 模糊PID计算的基准参数
typedef struct Struct_PID_Manage_Object_Fuzzy
{
    float kp;
    float ki;
    float kd;
    float kf;

    float error;
    float last_error;
    float before_last_error;
    float integral_error; // 误差积分累积
    float error_target;   // 前馈控制
    float err_change;     // 误差变化率

    float set_point;      // 目标值
    float now_point;      // 当前值
    float last_set_point; // 上一次目标值

    float integral_limit;      // 积分限幅
    float output_limit;        // 输出限幅
    float Integral_Separation; // 积分分离阈值
    float deadzone;            // 死区

    float p_out;  // p输出
    float i_out;  // i输出
    float d_out;  // d输出
    float f_out;  // 前馈f输出
    float output; // 总输出
} Struct_PID_Manage_Object_Fuzzy;

extern Struct_PID_Manage_Object_Fuzzy M3508s_DialPID;
extern Struct_PID_Manage_Object_Fuzzy M2006s_FricLPID;
extern Struct_PID_Manage_Object_Fuzzy M2006s_FricRPID;

extern positionpid_t M6020s_YawIPID;		//Yaw	
extern positionpid_t M6020s_Yaw_SpeedPID;	//Yaw�ٶ�PID
extern positionpid_t M6020s_YawOPID;		//Yaw
extern positionpid_t AutoAim_M6020s_YawIPID;
extern positionpid_t AutoAim_M6020s_YawOPID;   //Yaw����PID

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
extern float Position_PID(positionpid_t *pid_t, float target, float measured);
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max);
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit);
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float Kf, float MaxOutput, float Integral_Separation, float IntegralLimit);

extern float Position_PID_Pitch(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);
extern float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);
extern void Clear_PositionPIDData(positionpid_t *pid_t);
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);


extern One_Kalman_t Cloud_YAWODKalman;
extern One_Kalman_t Cloud_PITCHODKalman;

#endif
