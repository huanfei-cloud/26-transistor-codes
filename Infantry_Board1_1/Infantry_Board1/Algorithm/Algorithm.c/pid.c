/**
 * @file PID.c
 * @author Miraggio (w1159904119@gmail)，XYZ
 * @brief
 * @version 0.1
 * @date 2024-011-30
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "PID.h"

One_Kalman_t Cloud_YAWODKalman;
One_Kalman_t Cloud_PITCHODKalman;

// 限制PID参数范围
void limit_pid_params(float Kp, float Ki, float Kd) 
{

    if (Kp < 0.1f) Kp = 0.1f;
    if (Kp > 40.0f) Kp = 40.0f;
    if (Ki < 0.001f) Ki = 0.001f;
    if (Ki > 0.5f) Ki = 0.5f;
    if (Kd < 0.001f) Kd = 0.001f;
    if (Kd > 30.f) Kd = 30.f;
}


//限制绝对值
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}


//模糊pid的Pitch
float Position_PID_Pitch(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured)
{
	  float fuzzy_kf;
	  FuzzyComputation(fuzzy_t, pid_t->err, pid_t->err_last);
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
	  pid_t->error_target = pid_t->Target - pid_t->last_set_point;
	
		fuzzy_kf=pid_t->kf;

    pid_t->p_out = (pid_t->Kp + fuzzy_t->delta_kp) * pid_t->err;
    pid_t->i_out += (pid_t->Ki + fuzzy_t->delta_ki) * pid_t->err;
    pid_t->d_out = (pid_t->Kd + fuzzy_t->delta_kd) * (pid_t->err - pid_t->err_last);
    pid_t->f_out = fuzzy_kf * pid_t->error_target;
    if (pid_t->err >= pid_t->Integral_Separation)
		pid_t->i_out = 0;
	else
		//积分限幅
		abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

//模糊pid函数
float PID_Model4_Update(incrementalpid_t *pid,FUZZYPID_Data_t *PID, float _set_point, float _now_point)
{
		float fuzzy_kp;
		float fuzzy_ki;
		float fuzzy_kd;
		float fuzzy_kf;
	
		float network_kp;
		float network_ki;
		float network_kd;
	  float delta_error;
    pid->Target = _set_point;
    pid->Measured = _now_point;
    pid->err = pid->Target - pid->Measured;
    pid->error_target = pid->Target - pid->last_set_point;
	  delta_error = pid->err-pid->err_last;
		pid->integral_error += pid->err;
		FuzzyComputation (PID,(float)pid->err,(float)pid->err_last);
		fuzzy_kp=pid->Kp+PID->delta_kp*PID->qKp;
		fuzzy_ki=pid->Ki+PID->delta_ki*PID->qKi;
		fuzzy_kd=pid->Kd+PID->delta_kd*PID->qKd;	
		fuzzy_kf=pid->kf;
		
    pid->p_out =fuzzy_kp* pid->err;
    abs_limit(&pid->integral_error, pid->IntegralLimit);
    pid->i_out = fuzzy_ki * pid->integral_error;
    pid->d_out = fuzzy_kd * (pid->err - pid->err_last);
    pid->f_out = fuzzy_kf * pid->error_target;
  	pid->pwm=pid->p_out+pid->i_out+pid->d_out+pid->f_out;
    abs_limit(&pid->pwm, pid->MaxOutput);

    pid->err_last = pid->err;
    pid->last_set_point = pid->Target;

    return pid->pwm;
}

//增量式pid
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{
    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

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

/**
  * @brief  增量式pid清除
  * @param  incrementalpid_t
  * @retval void
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

//位置式PID算法，对偏差值进行累加积分。
float Position_PID(positionpid_t *pid_t, float target, float measured)
{

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
	  pid_t->error_target  = pid_t->Target - pid_t->last_set_point;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
	  pid_t->f_out = pid_t->kf * pid_t->error_target;
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
		pid_t->last_set_point  = pid_t->Target;
    return pid_t->pwm;
}

void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float Kf, float MaxOutput, float Integral_Separation, float IntegralLimit)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
	  pid_t->kf = Kf;
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_change = 0;
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

