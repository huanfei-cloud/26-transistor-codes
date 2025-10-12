/**
 * @file MA600.c
 * @author xhf
 * @brief 
 * @version 0.1
 * @date 2025-05-06
 * 
 */
#include "MA600_use.h"

/********全局变量定义********/
hMA600_TypeDef MA600_Roll;

/**
 * @brief 四个编码器初始化
 * @param None
 * @retval None
 */
void MA600sInit(void)
{
	MA600_HandleInit(&MA600_Roll,&hspi2, MA600_CS_Port, MA600_CS_Pin);

}


/**
 * @brief 获取roll轴的角度值
 * @param None
 * @retval None
 */

float MA600_Roll_Calc_Location(void)
{
	int32_t totalAngle;
	float totalcircles;
	//获取总圈数
	MA600_Roll.Circle = MA600_Get_MultiTurn(&MA600_Roll);
	//获取当前角度
	MA600_Roll.Angle = MA600_Get_Angle(&MA600_Roll);
	//计算总角度值
	totalAngle = MA600_Roll.Angle - MA600_Roll_InitAngle + MA600_Roll.Circle * 65535;
	//转化为圈数
	totalcircles = totalAngle / 65535;
	
	return totalcircles;
		
}






