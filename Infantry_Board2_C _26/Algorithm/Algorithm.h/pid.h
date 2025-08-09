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
//#include "typedef.h"
//#include <AddMath.h>
#include "math.h"
#include "FuzzyPID.h"

/**********PID对外数据接口************/

// pid控制方式选择
typedef enum
{
	pid_control_increase,
	pid_control_normal,
	pid_control_frontfeed,
	pid_control_frontfuzzy
}pid_control;


typedef struct incrementalpid_t
{
    float Target;         //设定目标值
    float Measured;       //测量值
    float err;            //本次偏差值
    float err_last;       //上一次偏差
    float err_beforeLast; //上上次偏差
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;            //各部分输出值
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
	  float error_target;   // 前馈控制
		float last_set_point; //上一次目标值
    float Kp;
    float Ki;
    float Kd; 
	  float Kf;         //Kp, Ki, Kd, Kf控制系数
    float p_out;
    float i_out;
    float d_out;               
	  float f_out;               //各部分输出值
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} positionpid_t;

extern positionpid_t M6020s_YawIPID;		//Yaw	
extern positionpid_t M6020s_Yaw_SpeedPID;	//Yaw速度PID
extern positionpid_t M6020s_YawOPID;		//Yaw
extern positionpid_t AutoAim_M6020s_YawIPID;
extern positionpid_t AutoAim_M6020s_YawOPID;   //Yaw自瞄PID

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
extern float Position_PID(positionpid_t *pid_t, float target, float measured);
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki, float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);

extern One_Kalman_t Cloud_YAWODKalman;
extern One_Kalman_t Cloud_PITCHODKalman;

extern void Clear_PositionPIDData(positionpid_t *pid_t);
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

extern float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

#endif
