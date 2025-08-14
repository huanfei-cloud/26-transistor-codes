/**
 * @file bsp_pid.c
 * @author xyz
 * @brief pid��ش���
 * @version 1.1
 * @date 2024-09-18
 * @update 2024-09-28 ����Ť��ǰ��
 * @copyright Transistor BUAA
 */

#include "pid.h"

/**
 * @brief ����ֵ����
 */
static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/**
 * @brief pid��ʼ��
 * @param pֵ
 * @param iֵ
 * @param dֵ
 */
void PID_Init(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _integral_limit, float _output_limit, float _deadzone)
{
    pid->kp = _kp;
    pid->ki = _ki;
    pid->kd = _kd;
    pid->kf = _kf;

    pid->error = 0;
    pid->last_error = 0;
    pid->before_last_error = 0;
    pid->integral_error = 0;

    pid->last_set_point = 0;
    pid->set_point = 0;
    pid->now_point = 0;

		pid->filtered_d = 0;
    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;
    pid->f_out = 0;
    pid->output = 0;

    pid->integral_limit = _integral_limit;
    pid->output_limit = _output_limit;

    pid->deadzone = _deadzone;
}

/**
 * @brief ����ʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float PID_Model1_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point)
{
    pid->set_point = _set_point;
    pid->now_point = _now_point;
    pid->error = pid->set_point - pid->now_point;
    pid->p_out = pid->kp * (pid->error - pid->last_error);
    pid->i_out = pid->ki * pid->error;
    pid->d_out = pid->kd * (pid->error - 2.0f * pid->last_error + pid->before_last_error);
    abs_limit(&pid->i_out, pid->integral_limit);
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    abs_limit(&pid->output, pid->output_limit);
    pid->before_last_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}
/**
 * @brief ��ͨʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float PID_Model2_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point)
{
    pid->set_point = _set_point;
    pid->now_point = _now_point;
    pid->error = pid->set_point - pid->now_point;
    pid->p_out = pid->kp * pid->error;
    pid->integral_error += pid->error;
    abs_limit(&pid->integral_error, pid->integral_limit);
    pid->i_out = pid->ki * pid->integral_error;
    pid->d_out = pid->kd * (pid->error - pid->last_error);
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief ǰ��ʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float PID_Model3_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point)
{
		pid->set_point = _set_point;
    pid->now_point = _now_point;
    pid->error = pid->set_point - pid->now_point;
		if(fabs(pid->error) < pid->deadzone)
			return 0;  //������С���������������λΪ����ֵ
    pid->error_target = pid->set_point - pid->last_set_point;
    pid->p_out = pid->kp * pid->error;
    pid->integral_error += pid->error;
    abs_limit(&pid->integral_error, pid->integral_limit);
    pid->i_out = pid->ki * pid->integral_error;
		float raw_d=pid->error - pid->last_error;
		pid->filtered_d = 0.2f*raw_d + 0.8f * pid->filtered_d;
		pid->d_out = pid->kd*pid->filtered_d;
    pid->f_out = pid->kf * pid->error_target;
  	pid->output=pid->p_out+pid->i_out+pid->d_out+pid->f_out;
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->error;
    pid->last_set_point = pid->set_point;

    return pid->output;
}

/**
 * @brief ǰ��ʽģ��PID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float PID_Model4_Update(struct Struct_PID_Manage_Object *pid,FUZZYPID_Data_t *PID, float _now_point, float _set_point)
{
		float fuzzy_kp;
		float fuzzy_ki;
		float fuzzy_kd;
		float fuzzy_kf;
    pid->set_point = _set_point;
    pid->now_point = _now_point;
    pid->error = pid->set_point - pid->now_point;
    pid->error_target = pid->set_point - pid->last_set_point;
		pid->integral_error += pid->error;
		FuzzyComputation (PID,(float)pid->error,(float)pid->last_error);
		fuzzy_kp=pid->kp+PID->delta_kp*PID->qKp;
		fuzzy_ki=pid->ki+PID->delta_ki*PID->qKi;
		fuzzy_kd=pid->kd+PID->delta_kd*PID->qKd;	
		fuzzy_kf=pid->kf;
	
    pid->p_out =fuzzy_kp* pid->error;
    
    abs_limit(&pid->integral_error, pid->integral_limit);
    pid->i_out = fuzzy_ki * pid->integral_error;
    pid->d_out = fuzzy_kd * (pid->error - pid->last_error);
    pid->f_out = fuzzy_kf * pid->error_target;
  	pid->output=pid->p_out+pid->i_out+pid->d_out+pid->f_out;
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->error;
    pid->last_set_point = pid->set_point;

    return pid->output;
}


/**
 * @brief pid��������
 * @param pֵ
 * @param iֵ
 * @param dֵ
 * @param fֵ
 */
void pid_change(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf)
{
    pid->kp = _kp;
    pid->ki = _ki;
    pid->kd = _kd;
    pid->kf = _kf;
}
