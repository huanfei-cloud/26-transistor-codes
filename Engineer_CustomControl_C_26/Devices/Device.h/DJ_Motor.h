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

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/
#include "pid.h"
#include "BSP_Can.h"

/* Exported types ------------------------------------------------------------*/

//ID规定
#define M2006_J2_ID 0x201
#define M2006_J3_ID 0x202
//减速比规定
#define M2006_RADIO 36

/**
 * @brief 电机处理结构体
 */
typedef struct 
{
	// 经Rx_Buffer中的数据转换得到的电机基础数据
	uint16_t can_id;
	int16_t omega;	  // 转速
	uint16_t encoder; // 编码器位置
	int16_t torque;	  
	uint8_t temperaure;

	int16_t i_send;			    // 发送电流/电压
	uint16_t pre_encoder;	  // 上次的编码器位置
	uint16_t now_encoder;	  // 这次的编码器位置
	uint32_t zero_encoder;  //上电时的零点encoder;
	int32_t delta_encoder;	// 编码器变化
	int32_t total_encoder;  // 编码器总值
	int32_t total_round;	  // 旋转圈数
	int32_t UpdateFrame;    //帧率
	int8_t UpdateFlag;
	int8_t zero_flag;
	
	int16_t target_v;				// 目标速度
	float target_location;	// 目标位置
                                 	
	struct Struct_PID_Manage_Object l_pid_object; // 对应角度环pid处理结构体
	struct Struct_PID_Manage_Object v_pid_object; // 对应速度环pid处理结构体
}DJ_Motors_t;

/********全局变量声明********/
extern DJ_Motors_t M2006s_J2;
extern DJ_Motors_t M2006s_J3;
/********函数声明********/
void motor_init(DJ_Motors_t *motor, uint16_t _motor_id);
void get_motor_data(DJ_Motors_t *motor,Can_Export_Data_t RxMessage);
void motor_velocity_change(DJ_Motors_t *motor,CAN_HandleTypeDef *hcan,enum pid_control model, float target);
void motor_location_change(DJ_Motors_t *motor, enum pid_control model, float target,float real);
float encoder_to_circle(int32_t encoder);
int32_t circle_to_encoder(float circle);

/* Exported functions -*/

/* Exported variables --------------------------------------------------------*/

#endif
