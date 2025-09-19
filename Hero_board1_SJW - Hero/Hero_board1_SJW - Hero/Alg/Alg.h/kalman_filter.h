#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "arm_math.h"

/*************һ�׿�����**************/
typedef struct
{
  float X_Last; // ��һʱ�̵����Ž��  X(k-1|k-1)
  float X_Mid;  // ��ǰʱ�̵�Ԥ����  X(k|k-1)
  float X_Now;  // ��ǰʱ�̵����Ž��  X(k|k)
  float P_Last; // ��һʱ�����Ž����Э����  P(k-1|k-1)
  float P_Mid;  // ��ǰʱ��Ԥ������Э����  P(k|k-1)
  float P_Now;  // ��ǰʱ�����Ž����Э����  P(k|k)
  float Kt;     // kalman����
  float A;      // ״̬ת�ƾ���
  float B;      // ���ƾ���
  float Q;      // ��������Э����
  float R;      // �۲�����Э����
  float H;
} One_Kalman_t;

void One_Kalman_Create(One_Kalman_t *Kal, float T_Q, float T_R);
float One_Kalman_Filter(One_Kalman_t *Kal, float Data);
/*************һ�׿����� END**************/

/*************���׿�����**************/
#define mat arm_matrix_instance_f32    // float
#define mat_64 arm_matrix_instance_f64 // double
#define mat_init arm_mat_init_f32      // �����ʼ��
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32 // �������ת��
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define Angle_limit 200        // �Ƕ�С��50����Ԥ��
#define PredictAngle_limit 250 // Ԥ��ֵ�޷�

#define Kf_Angle 0
#define Kf_Speed 1

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} Two_Kalman_Filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} Two_Kalman_Filter_Init_t;

typedef struct
{
  float Vision_Angle; // �Ӿ�--�Ƕ�
  float Vision_Speed; // �Ӿ�--�ٶ�
  float *Kf_result;   // ���������ֵ
  uint16_t Kf_Delay;  // ��������ʱ��ʱ

  struct
  {
    float Predicted_Factor;   // Ԥ���������
    float Predicted_SpeedMin; // Ԥ��ֵ��С�ٶ�
    float Predicted_SpeedMax; // Ԥ��ֵ����ٶ�
    float kf_delay_open;      // ��������ʱ����ʱ��
  } Parameter;
} Kalman_Data_t;

void Two_Kalman_Filter_Init(Two_Kalman_Filter_t *F, Two_Kalman_Filter_Init_t *I);
float *Two_Kalman_Filter_Calc(Two_Kalman_Filter_t *F, float signal1, float signal2);
/*************���׿����� END**************/

#endif
