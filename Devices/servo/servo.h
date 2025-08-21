/**
 * @file servo.h
 * @author ZTC
 * @brief
 * @version 0.1
 * @date 2025-08-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

typedef struct Servo_t
{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
  	int max_Angle;
	int min_Angle;
	float max_PWM_ratio;
	float min_PWM_ratio;
	int pwm_time_period;
} Servo_t;

extern Servo_t Servo_1;
extern Servo_t Servo_2;

void Servo_Init(Servo_t *Servo, TIM_HandleTypeDef *htim, uint32_t Channel,
                int max_Angle, int min_Angle, float max_PWM_ratio, float min_PWM_ratio,int pwm_time_period);
void TIM_PWM_Init(Servo_t *Servo);
void Servo_Set_Angle_Simple(Servo_t *Servo, int angle);
void Servo_Set_Angle(Servo_t *Servo, int angle);

#endif
