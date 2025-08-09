/**
 * @file PID.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "PID.h"

One_Kalman_t Cloud_YAWODKalman;
One_Kalman_t Cloud_PITCHODKalman;

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

//模糊pid的Yaw
float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured)
{
	  FuzzyComputation(fuzzy_t, pid_t->err, pid_t->err_last);
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;

    pid_t->p_out = (pid_t->Kp + fuzzy_t->deta_kp) * pid_t->err;
    pid_t->i_out += (pid_t->Ki + fuzzy_t->date_ki) * pid_t->err;
    pid_t->d_out = (pid_t->Kd + fuzzy_t->date_kd) * (pid_t->Measured - pid_t->err_last);
	  pid_t->f_out = pid_t->Kf * pid_t->error_target;
    if (pid_t->err >= pid_t->Integral_Separation)
		pid_t->i_out = 0;
	  else
		//积分限幅
		abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->Measured;
    pid_t->last_set_point = pid_t->Target;
    return pid_t->pwm;
}


float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{

    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    //	if(abs(pid_t->err)<0.1f)
    //		pid_t->err = 0.0f;
    //return 0;

    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}

void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->MaxOutput = MaxOutput;
    pid_t->IntegralLimit = IntegralLimit;
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

//位置式PID算法，对偏差值进行累加积分。
float Position_PID(positionpid_t *pid_t, float target, float measured)
{

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->Measured - pid_t->err_last;
    pid_t->error_target = pid_t->Target - pid_t->last_set_point;
	
    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->Measured - pid_t->err_last);
	  pid_t->f_out = pid_t->Kf * pid_t->error_target;
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
	  pid_t->last_set_point = pid_t->Target;
    return pid_t->pwm;
}

void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float Kf, float MaxOutput, float Integral_Separation, float IntegralLimit)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
		pid_t->Kf = Kf;
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
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

