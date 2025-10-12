/**
 * @file pid.h
 * @author xyz
 * @brief pid相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#ifndef PID_H
#define PID_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

extern uint8_t pset;
extern uint8_t iset;
extern uint8_t dset;

// pid控制方式选择
enum pid_control
{
	pid_control1,
	pid_control2,
	pid_control3,
	pid_control4
};

/**
 * @brief PID处理结构体
 */
struct Struct_PID_Manage_Object
{
   float kp;
   float ki;
   float kd;
   float kf;

   float error;
   float last_error;
   float before_last_error;
   float integral_error; // 误差积分累积
   float error_target;   // 前馈控制

   float set_point;      // 目标值
   float now_point;      // 当前值
   float last_set_point; // 上一次目标值

   float integral_limit; // 积分限幅
   float output_limit;   // 输出限幅
   float deadzone;       // 死区

   float p_out;  // p输出
   float i_out;  // i输出
   float d_out;  // d输出
   float f_out;  // 前馈f输出
   float output; // 总输出
};

/* Exported variables --------------------------------------------------------*/
extern void PID_Init(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf, float _integral_limit, float _output_limit, float _deadzone);
extern float PID_Model1_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point);
extern float PID_Model2_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point);
extern float PID_Model3_Update(struct Struct_PID_Manage_Object *pid, float _now_point, float _set_point);
extern void pid_change(struct Struct_PID_Manage_Object *pid, float _kp, float _ki, float _kd, float _kf);
extern float encoder_to_circle(int32_t encoder);
extern int32_t circle_to_encoder(float circle);
#endif
