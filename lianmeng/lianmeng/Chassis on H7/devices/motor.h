/**
 * @file motor.h
 * @author xyz
 * @brief motor相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/
#include "PID.h"
#include "bsp_fdcan.h"

/* Exported types ------------------------------------------------------------*/

// pid控制方式选择
enum pid_control
{
	pid_control1,
	pid_control2,
	pid_control3,
	pid_control4
};

/**
 * @brief 电机处理结构体
 */
struct Struct_MOTOR_Manage_Object
{
	// 经Rx_Buffer中的数据转换得到的电机基础数据
	int16_t omega;	  // 转速
	uint16_t encoder; // 编码器位置
	int16_t torque;	  //
	uint8_t temperaure;
	uint16_t can_id;

	int16_t i_send;		   // 发送电流/电压
	uint16_t pre_encoder;  // 上次的编码器位置
	uint16_t now_encoder;  // 这次的编码器位置
	uint32_t zero_encoder; // 上电时的零点encoder;
	int32_t delta_encoder; // 编码器变化
	int32_t total_encoder; // 编码器总值
	int32_t total_round;   // 旋转圈数
	int32_t UpdateFrame;   // 帧率
	int8_t UpdateFlag;
	int8_t zero_flag;

	int16_t target_v;							  // 目标速度
	float target_location;						  // 目标位置 // 对应速度环pid处理结构体
	struct Struct_PID_Manage_Object l_pid_object; // 对应角度环pid处理结构体
	struct Struct_PID_Manage_Object v_pid_object;
};

extern void motor_init(struct Struct_MOTOR_Manage_Object *motor, uint16_t _motor_id);

extern void motor_encoder_state_change(struct Struct_MOTOR_Manage_Object *motor);

extern void motor_velocity_change(struct Struct_MOTOR_Manage_Object *motor, enum pid_control model, float target);
extern void motor_location_change(struct Struct_MOTOR_Manage_Object *motor, enum pid_control model, float target);
extern float encoder_to_quanshu(int32_t encoder);
extern int32_t quanshu_to_encoder(float quanshu);
extern void get_motor_data(struct Struct_MOTOR_Manage_Object *motor, uint8_t *data);

/* Exported functions -*/

/* Exported variables --------------------------------------------------------*/

#endif
