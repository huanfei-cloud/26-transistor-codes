/**
 * @file PID.h
 * @author Miraggio (w1159904119@gmail.com)
 * @brief PID������ģ��ͷ�ļ�
 * @version 0.1
 * @date 2021-03-30
 * @copyright Copyright (c) 2021
 */

#ifndef ___PID_H
#define ___PID_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kalman_filter.h"
#include "math.h"
#include "FuzzyPID.h"

/**********PID���������ݽṹ�ӿ�************/

/**
 * @brief PID���Ʒ�ʽö��
 */
typedef enum
{
    pid_control_increase,   ///< ����ʽPID����
    pid_control_normal,      ///< ��׼λ��ʽPID����
    pid_control_frontfeed,   ///< ��ǰ����λ��ʽPID����
    pid_control_frontfuzzy  ///< ��ģ�����Ƶ�λ��ʽPID����
} pid_control;

/**
 * @brief ����ʽPID�������ṹ��
 */
typedef struct incrementalpid_t
{
    float Target;           ///< �趨Ŀ��ֵ
    float Measured;         ///< ����ֵ
    float err;              ///< ��ǰƫ��ֵ
    float err_last;         ///< ��һ��ƫ��
    float err_beforeLast;   ///< ���ϴ�ƫ��
    float Kp;               ///< ����ϵ��
    float Ki;               ///< ����ϵ��
    float Kd;               ///< ΢��ϵ��
    float p_out;            ///< ���������
    float i_out;            ///< ���������
    float d_out;            ///< ΢�������
    float pwm;              ///< PWM���ֵ
    uint32_t MaxOutput;     ///< ����޷�ֵ
    uint32_t IntegralLimit; ///< �������޷�ֵ
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured); ///< ����ʽPID���㺯��ָ��
} incrementalpid_t;

/**
 * @brief λ��ʽPID�������ṹ��
 */
typedef struct positionpid_t
{
    float Target;           ///< �趨Ŀ��ֵ
    float Measured;         ///< ����ֵ
    float err;              ///< ��ǰƫ��ֵ
    float err_last;         ///< ��һ��ƫ��
    float err_change;       ///< ƫ��仯��
    float error_target;     ///< Ŀ��ֵ�仯��������ǰ����
    float last_set_point;   ///< ��һ��Ŀ��ֵ
    float Kp;               ///< ����ϵ��
    float Ki;               ///< ����ϵ��
    float Kd;               ///< ΢��ϵ��
    float Kf;               ///< ǰ��ϵ��
    float p_out;            ///< ���������
    float i_out;            ///< ���������
    float d_out;            ///< ΢�������
    float f_out;            ///< ǰ�������
    float pwm;              ///< PWM���ֵ
    float MaxOutput;        ///< ����޷�ֵ
    float Integral_Separation; ///< ���ַ�����ֵ
    float IntegralLimit;    ///< �������޷�ֵ
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured); ///< λ��ʽPID���㺯��ָ��
} positionpid_t;

/**********�ⲿ��������************/

/// Yaw��λ�û�PID������
extern positionpid_t M6020s_YawIPID;

/// Yaw���ٶȻ�PID������
extern positionpid_t M6020s_Yaw_SpeedPID;

/// Yaw�������PID������
extern positionpid_t M6020s_YawOPID;

/// �Զ���׼Yaw��λ�û�PID������
extern positionpid_t AutoAim_M6020s_YawIPID;

/// �Զ���׼Yaw�������PID������
extern positionpid_t AutoAim_M6020s_YawOPID;

/**********��������************/

/**
 * @brief ����ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 */
extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

/**
 * @brief λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 */
extern float Position_PID(positionpid_t *pid_t, float target, float measured);

/**
 * @brief ��̨��תPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 */
extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);

/**
 * @brief �Ƕ�PID���������Զ�����ǶȻ��ƣ�
 * @param pid_t PID�������ṹ��ָ��
 * @param target Ŀ��Ƕ�ֵ
 * @param measured �����Ƕ�ֵ
 * @param ecd_max ���ص�����������ֵ������8192��Ӧ360�ȣ�
 * @return PID�������ֵ
 */
extern float Angle_PID(positionpid_t *pid_t, float target, float measured,float ecd_max);

/**
 * @brief ���̸���ģʽ���ƺ���
 * @param angle Ŀ��Ƕ�
 * @param start_flag ������־
 */
extern void chassis_follow_mode(float angle, uint8_t start_flag);

/**
 * @brief ��ʼ������ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param Kp ����ϵ��
 * @param Kd ΢��ϵ��
 * @param Ki ����ϵ��
 * @param MaxOutput ����������
 * @param IntegralLimit �������޷�ֵ
 */
extern void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);

/**
 * @brief ��ʼ��λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param Kp ����ϵ��
 * @param Kd ΢��ϵ��
 * @param Ki ����ϵ��
 * @param Kf ǰ��ϵ��
 * @param MaxOutput ����������
 * @param IntegralLimit �������޷�ֵ
 * @param Integral_Separation ���ַ�����ֵ
 */
extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki, float Kf, float MaxOutput, float IntegralLimit, float Integral_Separation);
extern float speed_angle_limit_pid(positionpid_t *pid_t, float speed_target, float speed_measured,float angle_measured);

/**********�������˲���ʵ������************/

/// Yaw�Ῠ�����˲���ʵ��
extern One_Kalman_t Cloud_YAWODKalman;

/// Pitch�Ῠ�����˲���ʵ��
extern One_Kalman_t Cloud_PITCHODKalman;

/**
 * @brief ���λ��ʽPID����������
 * @param pid_t PID�������ṹ��ָ��
 */
extern void Clear_PositionPIDData(positionpid_t *pid_t);

/**
 * @brief �������ʽPID����������
 * @param pid_t PID�������ṹ��ָ��
 */
extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

/**
 * @brief Yaw��ģ��λ��ʽPID������
 * @param pid_t PID�������ṹ��ָ��
 * @param fuzzy_t ģ��PID���ݽṹ��ָ��
 * @param target Ŀ��ֵ
 * @param measured ����ֵ
 * @return PID�������ֵ
 */
extern float Position_PID_Yaw(positionpid_t *pid_t, FUZZYPID_Data_t *fuzzy_t, float target, float measured);

#endif /* ___PID_H */