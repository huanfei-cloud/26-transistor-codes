/**
 * @file FuzzyPID.h
 * @author Why/xyz
 * @brief 这里的函数设计想法是写到PID的计算内部去，实现Fuzzy在线调整PID参数
 *		  需要调的数只有PID的最大改变范围
 * @version 0.1
 * @date 2024-10-21
 *
 */
#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

// 初始化结构体参数
#include "stm32h7xx_hal.h"

// 定义模糊PID结构体
typedef struct
{
	float delta_kp; // 比例值增量比例
	float delta_ki; // 积分值增量比例
	float delta_kd; // 微分值增量比例

	float error_maximum; // 输出值的误差上限
	float error_minimum; // 输出值的误差下限

	float qKp; // kp增量的修正范围
	float qKi; // ki增量的修正范围
	float qKd; // kd增量的修正范围

	float error_map[2]; // error/d_error模糊化得到的值

	float error_membership_degree[2]; // error模糊化隶属度
	int8_t error_membership_index[2]; // error模糊化索引

	float d_error_membership_degree[2]; // d_error模糊化隶属度
	int8_t d_error_membership_index[2]; // d_error模糊化索引

} FUZZYPID_Data_t;
extern void Linear_Quantization(FUZZYPID_Data_t *PID, float thisError, float lastError, float *qValue);
extern void Membership_Calc(float *ms, float qv, int8_t *index);
extern void FuzzyComputation(FUZZYPID_Data_t *PID, float thisError, float lastError);
extern void fuzzy_init(FUZZYPID_Data_t *PID, float _maximum, float _minimum, float _qkp, float _qki, float _qkd);

extern FUZZYPID_Data_t fuzzy_pid_shoot_up;
extern FUZZYPID_Data_t fuzzy_pid_shoot_down;
extern FUZZYPID_Data_t fuzzy_pid_bullet_v;
extern FUZZYPID_Data_t fuzzy_pid_bullet_l;
extern FUZZYPID_Data_t fuzzy_pid_pitch_v;
extern FUZZYPID_Data_t fuzzy_pid_pitch_l;
extern FUZZYPID_Data_t fuzzy_pid_yaw_v;
extern FUZZYPID_Data_t fuzzy_pid_yaw_l;
#endif
