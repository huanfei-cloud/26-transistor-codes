/**
 * @file FuzzyPID.c
 * @author Why/xyz
 * @brief 这里的函数设计想法是写到PID的计算内部去，实现Fuzzy在线调整PID参数
 *		  需要调的数只有PID的最大改变范围
 * @version 0.1
 * @date 2023-09-28
 *
 */

#include "FuzzyPID.h"

FUZZYPID_Data_t fuzzy_pid_shoot_F1 = {0};
FUZZYPID_Data_t fuzzy_pid_shoot_F2 = {0};
FUZZYPID_Data_t fuzzy_pid_shoot_F3 = {0};
FUZZYPID_Data_t fuzzy_pid_shoot_B1 = {0};
FUZZYPID_Data_t fuzzy_pid_shoot_B2 = {0};
FUZZYPID_Data_t fuzzy_pid_shoot_B3 = {0};

FUZZYPID_Data_t fuzzy_pid_bullet_v = {0};
FUZZYPID_Data_t fuzzy_pid_bullet_l = {0};
FUZZYPID_Data_t fuzzy_pid_pitch_in = {0};
FUZZYPID_Data_t fuzzy_pid_pitch_out = {0};
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

/* 规则库 */
static const float ruleKp[7][7] = {
    PB, PB, PM, PM, PS, ZO, ZO,
    PB, PB, PM, PS, PS, ZO, NS,
    PM, PM, PM, PS, ZO, NS, NS,
    PM, PM, PS, ZO, NS, NM, NM,
    PS, PS, ZO, NS, NS, NM, NM,
    PS, ZO, NS, NM, NM, NM, NB,
    ZO, ZO, NM, NM, NM, NB, NB};

static const float ruleKi[7][7] = {
    NB, NB, NM, NM, NS, ZO, ZO,
    NB, NB, NM, NS, NS, ZO, ZO,
    NB, NM, NS, NS, ZO, PS, PS,
    NM, NM, NS, ZO, PS, PM, PM,
    NS, NS, ZO, PS, PS, PM, PB,
    ZO, ZO, PS, PS, PM, PB, PB,
    ZO, ZO, PS, PM, PM, PB, PB};

static const float ruleKd[7][7] = {
    PS, NS, NB, NB, NB, NM, PS,
    PS, NS, NB, NM, NM, NS, ZO,
    ZO, NS, NM, NM, NS, NS, ZO,
    ZO, NS, NS, NS, NS, NS, ZO,
    ZO, ZO, ZO, ZO, ZO, ZO, ZO,
    PB, NS, PS, PS, PS, PS, PB,
    PB, PM, PM, PM, PS, PS, PB};

FUZZYPID_Data_t FuzzyPID_Pitch = FUZZYPID_Pitch_GroupInit;
#undef FUZZYPID_Pitch_GroupInit

void fuzzy_init(FUZZYPID_Data_t *PID, float _maximum, float _minimum, float _qkp, float _qki, float _qkd)
{
  PID->delta_kp = 0;
  PID->delta_ki = 0;
  PID->delta_kd = 0;

  PID->error_maximum = _maximum;
  PID->error_minimum = _minimum;

  PID->qKp = _qkp;
  PID->qKi = _qki;
  PID->qKd = _qkd;
}

/**
  * @brief  把误差和误差的变化量转化到-3到3的论域上，先算e再算ec
  * @param  FUZZYPID_Data_t *PID --对应的FuzzyPID结构体指针
      float thisError      --这次的误差
      float lastError       --上一次的误差
      float *qValue         --存E和EC量化的数组首地址指针
  * @retval None
  */
void Linear_Quantization(FUZZYPID_Data_t *PID, float thisError, float lastError, float *qValue)
{
  float deltaError;
  deltaError = thisError - lastError; // 计算偏差增量

  // E和EC的量化
  qValue[0] = 3.0f * thisError * 2 / (PID->error_maximum - PID->error_minimum);
  qValue[1] = 3.0f * deltaError / (PID->error_maximum - PID->error_minimum);
  if (qValue[0] >= 5)
  {
    qValue[0] = 5;
  }
  if (qValue[0] <= -5)
  {
    qValue[0] = -5;
  }
  if (qValue[1] >= 5)
  {
    qValue[1] = 5;
  }
  if (qValue[1] <= -5)
  {
    qValue[1] = -5;
  }
}
/**
  * @brief  隶属度计算函数，使用纯线性的折线
  * @param  float *ms 		--对应模糊子集的隶属度
      float qv      	--量化后的模糊量
      int *index      --根据大小存规则表下标的数组首地址
  * @retval None
  */
void Membership_Calc(float *ms, float qv, int8_t *index)
{
  if ((qv >= NB) && (qv < NM))
  {
    index[0] = 0;
    index[1] = 1;
    ms[0] = -2 - qv;
    ms[1] = qv + 3.0f;
  }
  else if ((qv >= NM) && (qv < NS))
  {
    index[0] = 1;
    index[1] = 2;
    ms[0] = -1 * qv - 1.0f; // y=-1x-1.0
    ms[1] = qv + 2.0f;      // y=0.5x+2.0
  }
  else if ((qv >= NS) && (qv < ZO))
  {
    index[0] = 2;
    index[1] = 3;
    ms[0] = -1 * qv;   // y=-1x
    ms[1] = qv + 1.0f; // y=0.5x+1.0
  }
  else if ((qv >= ZO) && (qv < PS))
  {
    index[0] = 3;
    index[1] = 4;
    ms[0] = -1 * qv + 1.0f; // y=-1x+1.0
    ms[1] = qv;             // y=0.5x
  }
  else if ((qv >= PS) && (qv < PM))
  {
    index[0] = 4;
    index[1] = 5;
    ms[0] = -1 * qv + 2.0f; // y=-1x+2.0
    ms[1] = qv - 1.0f;      // y=0.5x-1.0
  }
  else if ((qv >= PM) && (qv <= PB))
  {
    index[0] = 5;
    index[1] = 6;
    ms[0] = -1 * qv + 3.0f; // y=-1x+3.0
    ms[1] = qv - 2.0f;      // y=0.5x-2.0
  }
}

/**
  * @brief 更新FuzzyPID输出的PID参数的改变值
  * @param  FUZZYPID_Data_t *PID --对应的模糊PID结构体
      float thisError       --这次的误差
      float lastError       --上次的误差
  * @retval None
  */
void FuzzyComputation(FUZZYPID_Data_t *PID, float thisError, float lastError)
{
  /* pid增量值 */
  float pidvalue[3] = {0};

  /* 量化 */
  Linear_Quantization(PID, thisError, lastError, PID->error_map);
  // 计算e的隶属度和索引 */
  Membership_Calc(PID->error_membership_degree, PID->error_map[0], PID->error_membership_index);
  /* 计算ec的隶属度和索引 */
  Membership_Calc(PID->d_error_membership_degree, PID->error_map[1], PID->d_error_membership_index);

  pidvalue[0] += PID->error_membership_degree[0] * PID->d_error_membership_degree[0] * ruleKp[PID->error_membership_index[0]][PID->d_error_membership_index[0]];
  pidvalue[0] += PID->error_membership_degree[0] * PID->d_error_membership_degree[1] * ruleKp[PID->error_membership_index[0]][PID->d_error_membership_index[1]];
  pidvalue[0] += PID->error_membership_degree[1] * PID->d_error_membership_degree[0] * ruleKp[PID->error_membership_index[1]][PID->d_error_membership_index[0]];
  pidvalue[0] += PID->error_membership_degree[1] * PID->d_error_membership_degree[1] * ruleKp[PID->error_membership_index[1]][PID->d_error_membership_index[1]];

  pidvalue[1] += PID->error_membership_degree[0] * PID->d_error_membership_degree[0] * ruleKi[PID->error_membership_index[0]][PID->d_error_membership_index[0]];
  pidvalue[1] += PID->error_membership_degree[0] * PID->d_error_membership_degree[1] * ruleKi[PID->error_membership_index[0]][PID->d_error_membership_index[1]];
  pidvalue[1] += PID->error_membership_degree[1] * PID->d_error_membership_degree[0] * ruleKi[PID->error_membership_index[1]][PID->d_error_membership_index[0]];
  pidvalue[1] += PID->error_membership_degree[1] * PID->d_error_membership_degree[1] * ruleKi[PID->error_membership_index[1]][PID->d_error_membership_index[1]];

  pidvalue[2] += PID->error_membership_degree[0] * PID->d_error_membership_degree[0] * ruleKd[PID->error_membership_index[0]][PID->d_error_membership_index[0]];
  pidvalue[2] += PID->error_membership_degree[0] * PID->d_error_membership_degree[1] * ruleKd[PID->error_membership_index[0]][PID->d_error_membership_index[1]];
  pidvalue[2] += PID->error_membership_degree[1] * PID->d_error_membership_degree[0] * ruleKd[PID->error_membership_index[1]][PID->d_error_membership_index[0]];
  pidvalue[2] += PID->error_membership_degree[1] * PID->d_error_membership_degree[1] * ruleKd[PID->error_membership_index[1]][PID->d_error_membership_index[1]];

  /* pid增量修正 */
  // 得到的是pid改变的比例，需乘以qkp/qki/qkd发挥作用
  PID->delta_kp = pidvalue[0] / 3;
  PID->delta_ki = pidvalue[1] / 3;
  PID->delta_kd = pidvalue[2] / 3;
}
