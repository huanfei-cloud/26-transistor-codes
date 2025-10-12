/**
 * @file Arm_Control.c
 * @author xhf
 * @brief 机械臂控制
 * @version 1.0
 * @date 2025-08-21
 * @copyright Transistor BUAA
 */

#include "Arm_Control.h"

/********变量定义********/
CustomControl_Data_t  CustomControl_Data;

/**
 * @brief 自定义控制器电机初始化
 * @param None
 * @param None
 */
void CustomControl_Motor_Init(void)
{
    motor_init(&M2006s_J2,0x201);
	  motor_init(&M2006s_J3,0x202);	  
}

/**
 * @brief 自定义控制器数据获取
 * @param None
 * @param None
 */
void CustomControl_Set_Location(void)
{
	//J1电机控制数据来自MA600编码器
	CustomControl_Data.J1_targetlocation = MA600s_J1.Angle * 8191.0f / 65535.0f;
	//J2电机控制数据来自M2006电机
	CustomControl_Data.J2_targetlocation = M2006s_J2.encoder;
	//J3电机控制数据来自M2006电机
	CustomControl_Data.J3_targetlocation = M2006s_J3.encoder;
	//J4、J5、J6电机控制数据来自IMU
	N100_Read();
	CustomControl_Data.J4_targetlocation = N100_Angle.Yaw * 8191.0f / 360.0f;
	CustomControl_Data.J5_targetlocation = N100_Angle.Pitch * 8191.0f / 360.0f;
	CustomControl_Data.J6_targetlocation = N100_Angle.Roll * 8191.0f / 360.0f;
}
