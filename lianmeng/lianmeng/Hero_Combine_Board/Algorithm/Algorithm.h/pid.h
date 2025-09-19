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

/**********PID对外数据接口************/
typedef struct incrementalpid_t
{
    float Target;         //设定目标值
    float Measured;       //测量值
    float err;            //本次偏差值
    float err_last;       //上一次偏差
    float err_beforeLast; //上上次偏差
	  float last_set_point; // 上一次目标值
	  float error_target;   // 前馈控制
	  float integral_error; // 误差积分累积
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
	  float kf;
    float p_out;
    float i_out;
    float d_out;            //各部分输出值
	  float f_out;  // 前馈f输出
    float pwm;              //pwm输出
    uint32_t MaxOutput;     //输出限幅
    uint32_t IntegralLimit; //积分限幅
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;

typedef struct positionpid_t
{
    float Target;     //设定目标值
    float Measured;   //测量值
    float err;        //本次偏差值
    float err_last;   //上一次偏差
    float err_change; //误差变化率
	  float last_set_point; // 上一次目标值
	  float error_target;   // 前馈控制
	  float integral_error; // 误差积分累积
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
	  float kf;
    float p_out;
    float i_out;
    float d_out;               //各部分输出值
	float f_out;
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;

//模糊PID计算的基准参数
typedef struct Struct_PID_Manage_Object_Fuzzy
{
   float Kp;
   float Ki;
   float Kd;
   float Kf;
	 float Kc;   //????

   float error;
   float last_error;
   float before_last_error;
   float integral_error; // 误差积分累积
   float error_target;   // 前馈控制
	 float err_change; //误差变化率

   float Target;      // 目标值
   float Measured;      // 当前值
   float last_set_point; // 上一次目标值

   float integral_limit; // 积分限幅
   float output_limit;   // 输出限幅
	 float Integral_Separation; //积分分离阈值
   float deadzone;       // 死区

   float p_out;  // p输出
   float i_out;  // i输出
   float d_out;  // d输出
   float f_out;  // 前馈f输出
	 float feedforward_CONSTANT_out;
   float output; // 总输出
}Struct_PID_Manage_Object_Fuzzy;

extern Struct_PID_Manage_Object_Fuzzy M6020s_YawIPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_PitchIPID;		//Pitch	
extern Struct_PID_Manage_Object_Fuzzy M6020s_Yaw_SpeedPID;	//Yaw速度PID
extern Struct_PID_Manage_Object_Fuzzy M6020s_YawOPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_PitchOPID;		//Pitch	
extern Struct_PID_Manage_Object_Fuzzy M6020s_AimYawOPID;		//Yaw
extern Struct_PID_Manage_Object_Fuzzy M6020s_AimYawIPID;		//Yaw

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
extern float Position_PID(positionpid_t *pid_t, float target, float measured);
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki,float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);

extern float PID_Model4_Update(Struct_PID_Manage_Object_Fuzzy *pid,FUZZYPID_Data_t *PID, float _set_point, float _now_point);
extern void PID_Model4_Update_PIDInit(Struct_PID_Manage_Object_Fuzzy *pid_t, float Kp, float Ki, float Kd,float Kf,float Kc, uint32_t MaxOutput, uint32_t IntegralLimit, uint32_t Integral_Separation);
extern float Position_PID_Dial(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

extern One_Kalman_t Cloud_YAWODKalman;
extern One_Kalman_t Cloud_PITCHODKalman;

extern float Cloud_IMUPITCHIPID(positionpid_t *pid_t, float target, float measured);
extern float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUYAWOPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUYAWIPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUPITCHOPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_VisionIMUPITCHIPID(positionpid_t *pid_t, float target, float measured);
extern void Clear_PositionPIDData(positionpid_t *pid_t);
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);
extern float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured);
extern float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured);

#endif
