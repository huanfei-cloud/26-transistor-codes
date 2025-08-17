/**
 * @file PID.c
 * @author xyz
 * @brief pid��ش���
 * @version 1.1
 * @date 2024-09-18
 * @update 2024-09-28 ����Ť��ǰ��
 * @copyright Transistor BUAA
 */

#include "tpid.h"

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
void TPID_Init(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _integral_limit, float _output_limit, float _deadzone)
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
 * @brief pid��������
 * @param pֵ
 * @param iֵ
 * @param dֵ
 * @param fֵ
 */
void Tpid_change(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf)
{
    pid->kp = _kp;
    pid->ki = _ki;
    pid->kd = _kd;
    pid->kf = _kf;
}


/**
 * @brief ǰ��ʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float T_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point)
{
    pid->set_point = _set_point;
    pid->now_point = _now_point;
    pid->error = pid->set_point - pid->now_point;
    pid->error_target = pid->set_point - pid->last_set_point;
    pid->p_out = pid->kp * pid->error;
    pid->integral_error += pid->error;
    abs_limit(&pid->integral_error, pid->integral_limit);
    pid->i_out = pid->ki * pid->integral_error;
    pid->d_out = pid->kd * (pid->error - pid->last_error);
    pid->f_out = pid->kf * pid->error_target;
  	pid->output=pid->p_out+pid->i_out+pid->d_out+pid->f_out;
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->error;
    pid->last_set_point = pid->set_point;

    return pid->output;
}

/**
 * @brief pid��ʼ��
 * @param pֵ
 * @param iֵ
 * @param dֵ
 * @param fֵ
 */
void Position_PIDInit(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _output_limit, float _integral_limit)
{
    pid->kp = _kp;
    pid->ki = _ki;
    pid->kd = _kd;
		pid->kf = _kf;
    pid->output_limit = _output_limit;
    pid->integral_limit = _integral_limit;
    pid->p_out = 0;
    pid->d_out = 0;
    pid->i_out = 0;
    pid->error = 0;
    pid->last_error = 0;
	  pid->error_target = 0;
    pid->output = 0;
    pid->now_point = 0;
    pid->set_point = 0;
}
/**
 * @brief λ��ʽPID����
 * @param pid����ṹ��
 * @param ��ǰֵ
 * @param Ŀ��ֵ
 * @return ���ֵ
 */
float Position_PID(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point)
{

    pid->set_point = (float)_set_point;
    pid->now_point = (float)_now_point;
    pid->error = pid->set_point - pid->now_point;
    pid->error_target = pid->set_point - pid->last_set_point;
	
    pid->p_out = pid->kp * pid->error;
    pid->i_out += pid->ki * pid->error;
    pid->d_out = pid->kd * (pid->now_point - pid->last_error);
	  pid->f_out = pid->kf * pid->error_target;
    //�����޷�
    abs_limit(&pid->i_out, pid->integral_limit); 

    pid->output = (pid->p_out + pid->i_out + pid->d_out + pid->f_out);

    //����޷�
    abs_limit(&pid->output, pid->output_limit);

    pid->last_error = pid->now_point;
	  pid->last_set_point = pid->set_point;
    return pid->output;
}
