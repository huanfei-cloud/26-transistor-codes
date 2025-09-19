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

//模糊PID
float PID_Model4_Update(Struct_PID_Manage_Object_Fuzzy *pid,FUZZYPID_Data_t *PID, float _set_point, float _now_point)
{
		float fuzzy_kp;
		float fuzzy_ki;
		float fuzzy_kd;
		float fuzzy_kf;
	  int8_t zf_flag;
	
    pid->Target = _set_point;
    pid->Measured = _now_point;
    pid->error = pid->Target - pid->Measured;
	  
    if(pid->Target - pid->last_set_point<0)zf_flag =-1;
	  else if(pid->Target - pid->last_set_point>0)zf_flag =1;
	
    pid->error_target = zf_flag*fabs(pid->Target - pid->last_set_point)*(fabs(pid->Target - pid->last_set_point));
		pid->integral_error += pid->error;
		FuzzyComputation (PID,(float)pid->error,(float)pid->last_error);
		fuzzy_kp=pid->Kp+PID->delta_kp*PID->qKp;
		fuzzy_ki=pid->Ki+PID->delta_ki*PID->qKi;
		fuzzy_kd=pid->Kd+PID->delta_kd*PID->qKd;	
		fuzzy_kf=pid->Kf;
	
    pid->p_out =fuzzy_kp* pid->error;
    
    abs_limit(&pid->integral_error, pid->integral_limit );
    pid->i_out = fuzzy_ki * pid->integral_error;
    pid->d_out = fuzzy_kd * (pid->error - pid->last_error);
    pid->f_out = fuzzy_kf * pid->error_target;
  	pid->output = pid->p_out+pid->i_out+pid->d_out+pid->f_out+pid->feedforward_CONSTANT_out;
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->error;
    pid->last_set_point = pid->Target;

    return pid->output;
}

void PID_Model4_Update_PIDInit(Struct_PID_Manage_Object_Fuzzy *pid_t, float Kp, float Ki, float Kd,float Kf,float Kc, uint32_t MaxOutput, uint32_t IntegralLimit, uint32_t Integral_Separation)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
	  pid_t->Kf = Kf;
	  pid_t->Kc = Kc;
    pid_t->output_limit  = MaxOutput;
    pid_t->integral_limit  = IntegralLimit;
	  pid_t->Integral_Separation = Integral_Separation;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->error = 0;
    pid_t->last_error  = 0;
    pid_t->before_last_error  = 0;
    pid_t->output = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
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
    pid_t->err_change = pid_t->err - pid_t->err_last;
	  pid_t->error_target = pid_t->Target - pid_t->last_set_point ; 

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
	  pid_t->f_out = pid_t->kf * pid_t->error_target;
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out + pid_t->f_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput );

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd,float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation)
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

/* 加上模糊增量的专用pid，相比于一般位置pid多了模糊增量 */
float Position_PID_Dial(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured)
{
	//FuzzyComputation(fuzzy_t, pid_t->err, pid_t->err_last);
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = (pid_t->Kp + fuzzy_t->delta_kp) * pid_t->err;
    pid_t->i_out += (pid_t->Ki + fuzzy_t->delta_ki) * pid_t->err;
    pid_t->d_out = (pid_t->Kd + fuzzy_t->delta_kd) * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

