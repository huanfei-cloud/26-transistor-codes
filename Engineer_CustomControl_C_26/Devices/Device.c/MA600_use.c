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
hMA600_TypeDef MA600s_J1;

/**
 * @brief 编码器初始化
 * @param None
 * @retval None
 */
void MA600sInit(void)
{
	MA600_HandleInit(&MA600s_J1,&hspi2, MA600_CS_Port, MA600_CS_Pin);

}

/**
 * @brief 获取J1电机的控制位置
 * @param None
 * @retval None
 */
void MA600_J1_Location(void)
{
	 MA600_Get_Angle(&MA600s_J1);
	
}






