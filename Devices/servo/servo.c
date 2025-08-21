/**
 * @file servo.c
 * @author ZTC
 * @brief	舵机控制库	
 * 先初始化舵机，再初始化定时器以及PWM通道，最后可以设定舵机角度以及速度
 * 在cube中配置时，将TIM定时器对应的通道调到PWM generation CHx模式
 * prescaler为定时器挂载的总线频率*10**（-6）-1，如总线APB1频率168Mhz，则prescaler=167
 * 由电机的额定频率设定重载值counter period，counter period = 总线频率/额定频率/(prescaler+1)
 * pulse为设定的初始比较寄存器CCR值，一般为2000或小于counter period值即可
 * 
 * @version 1.0
 * @date 2025-08-20
 *
 * @copyright Copyright (c) 2025
 *
 */

 
#include "servo.h"

//默认为180度舵机，周期20ms，0.5ms对应-90度，1.5ms对应0，2.5ms对应90度
Servo_t Servo_1 = {
    .htim = NULL,
    .Channel = 0,
    .max_Angle = 90,
    .min_Angle = -90,
    .max_PWM_ratio = 0.125f,  // 2.5ms/20ms
    .min_PWM_ratio = 0.025f,  // 0.5ms/20ms
	.pwm_time_period = 20,
};
Servo_t Servo_2 = {
    .htim = NULL,
    .Channel = 0,
    .max_Angle = 90,
    .min_Angle = -90,
    .max_PWM_ratio = 0.125f,  // 2.5ms/20ms
    .min_PWM_ratio = 0.025f,  // 0.5ms/20ms
	.pwm_time_period = 20,
};

 /**
 * @name   Servo_Init
 * @brief  初始化舵机，内容是其所用的定时器以及其通道，转动角度，最大最小占空比
 * @param  TIM_HandleTypeDef *htim    &htim1
 * 		   Channel                    TIM_CHANNEL_1
 * @retval none
 * @attention
 */
void Servo_Init(Servo_t *Servo,TIM_HandleTypeDef *htim,uint32_t Channel
				,int max_Angle,int min_Angle,float max_PWM_ratio,float min_PWM_ratio,int pwm_time_period)
{
	Servo->htim = htim;
	Servo->Channel = Channel;
	Servo->max_Angle = max_Angle;
	Servo->min_Angle = min_Angle;
	Servo->max_PWM_ratio = max_PWM_ratio;
	Servo->min_PWM_ratio = min_PWM_ratio;
	Servo->pwm_time_period = pwm_time_period;
}

 /**
 * @name   TIM__PWM_Init
 * @brief  初始化定时器以及PWM通道
 * @retval none
 */
void TIM_PWM_Init(Servo_t *Servo)
{
	if (Servo->htim == NULL) return;
	HAL_TIM_Base_Start(Servo->htim);
	HAL_TIM_PWM_Start(Servo->htim,Servo->Channel);
}

 /**
 * @name   Servo_Set_Angle_Simple
 * @brief  在默认角度模式下，设定舵机的角度
 * @retval none
 * @attention angle的单位为度
 */
void Servo_Set_Angle_Simple(Servo_t *Servo, int angle)
{
	if (Servo->htim == NULL) return;  // 空指针检查
 	// 角度限幅
    if (angle > Servo->max_Angle) angle = Servo->max_Angle;
    if (angle < Servo->min_Angle) angle = Servo->min_Angle;
    
    // 计算CCR值 (20ms周期，0.5-2.5ms脉宽)
    float pulse_width = 500.0f + (angle - Servo->min_Angle) * 
                   (2000.0f / (Servo->max_Angle - Servo->min_Angle));
    
    uint32_t ccr = (uint32_t)(pulse_width / 20000.0f * Servo->htim->Init.Period);
    __HAL_TIM_SET_COMPARE(Servo->htim, Servo->Channel, ccr);
}

 /**
 * @name   Servo_Set_Angle
 * @brief  设定舵机的角度
 * @retval none
 * @attention angle的单位为度
 */
void Servo_Set_Angle(Servo_t *Servo, int angle)
{
	if (Servo->htim == NULL) return;
    // 角度限幅
    if (angle > Servo->max_Angle) angle = Servo->max_Angle;
    if (angle < Servo->min_Angle) angle = Servo->min_Angle;
    
    // 计算角度到时间的转换率
    float rate_angle_to_time = (Servo->max_PWM_ratio - Servo->min_PWM_ratio) * 
                               Servo->pwm_time_period / 
                               (Servo->max_Angle - Servo->min_Angle);
    
    // 计算脉宽(ms)并转换为CCR值
    float pulse_width = Servo->min_PWM_ratio * Servo->pwm_time_period + 
                       (angle - Servo->min_Angle) * rate_angle_to_time;
    
    uint32_t ccr = (uint32_t)((pulse_width / Servo->pwm_time_period) * 
                            Servo->htim->Init.Period + 0.5f);
    
    __HAL_TIM_SET_COMPARE(Servo->htim, Servo->Channel, ccr);
}

/*
void Servo_Set_Angle(Servo_t Servo, int angle, int pwm_time_period)
{
	static int ccr = 0;
	static float rate_angle_to_time = 90;
	rate_angle_to_time = (Servo.max_Angle-Servo.min_Angle)/(Servo.max_PWM_ratio-Servo.min_PWM_ratio)*Servo.htim->Init.Period;
	ccr = ((angle*rate_angle_to_time+0.5)/pwm_time_period)*(Servo.htim->Init.Period);
	__HAL_TIM_SET_COMPARE(&Servo.htim,Servo.Channel,(int)ccr);
}*/
