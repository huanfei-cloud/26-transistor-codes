/**
 * @file SpeedRamp.c
 * @author Miraggio (w1159904119@gmail)
 * @brief
 * @version 0.1
 * @date 2021-03-30
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "SpeedRamp.h"

/**
 * @brief  斜坡函数计算
 * @param	斜坡函数结构体
 * @retval None
 */
int16_t SpeedRampCalc(SpeedRamp_t *SpeedRamp)
{

	SpeedRamp->count += SpeedRamp->rate;
	if (SpeedRamp->count < SpeedRamp->mincount)
	{
		SpeedRamp->count = SpeedRamp->mincount;
	}
	if (SpeedRamp->count > SpeedRamp->maxcount)
	{
		SpeedRamp->count = SpeedRamp->maxcount;
	}
	return SpeedRamp->count;
}

/**
 * @brief  斜坡计数值归零
 * @param	斜坡函数结构体
 * @retval None
 */
void CountReset(SpeedRamp_t *SpeedRamp)
{

	if (fabs(SpeedRamp->count) < fabs(SpeedRamp->rate))
	{
		SpeedRamp->count = 0;
	}
	else
	{

		SpeedRamp->count -= SpeedRamp->count * 0.2f;
	}
}

/**
 * @brief  斜坡函数,使目标输出值缓慢等于期望值
 * @param  期望最终输出,当前输出,变化速度(越大越快)
 * @retval 当前输出
 * @attention
 */
float RAMP_float(float final, float now, float ramp)
{
	float buffer = 0; // 用于存储最终输出与当前输出的差值

	buffer = final - now; // 计算期望值与当前值的差

	if (buffer > 0) // 如果差值为正，表示当前值小于期望值
	{
		if (buffer > ramp) // 如果差值大于变化速度
		{
			now += ramp; // 当前值增加变化速度
		}
		else // 如果差值小于或等于变化速度
		{
			now += buffer; // 当前值直接增加差值，达到期望值
		}
	}
	else // 如果差值为负，表示当前值大于期望值
	{
		if (buffer < -ramp) // 如果差值小于负的变化速度
		{
			now += -ramp; // 当前值减少变化速度
		}
		else // 如果差值大于或等于负的变化速度
		{
			now += buffer; // 当前值直接减少差值，达到期望值
		}
	}

	return now; // 返回更新后的当前值
}
