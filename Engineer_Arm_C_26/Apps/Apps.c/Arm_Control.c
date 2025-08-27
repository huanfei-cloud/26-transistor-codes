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
Arm_Data_t  Arm_Data;

/**
 * @brief 机械臂电机初始化
 * @param None
 * @param None
 */
void Arm_Motor_Init(void)
{
    motor_init(&M2006_roll,0x205);
	  
	  //位置环PID初始化
	  PID_Init(&(M2006_roll.l_pid_object), 0.032f, 0.0001f, 0.01f, 0.0f,10000, 30000, 10);
	  //速度环PID初始化
	  PID_Init(&(M2006_roll.v_pid_object), 2.0f, 0.001f, 1.0f, 0.0f,10000, 30000, 10); 
	
	  Arm_Data.roll_targetlocation = 12;
}

/**
 * @brief 机械臂电机目标值计算
 * @param None
 * @param None
 */

void Arm_Target_Calc(void)
{
    M2006_roll.target_location = Arm_Data.roll_targetlocation;

}
/**
 * @brief 电机电流输出函数
 * @param None
 * @param None
 */
void Arm_Out(void)
{
	
	//roll真实值获取
	Arm_Data.roll_reallocation = MA600_Roll_Calc_Location();
	//目标值获取
	Arm_Target_Calc();
	
  motor_location_change(&M2006_roll, pid_control3,circle_to_encoder(Arm_Data.roll_reallocation),circle_to_encoder(M2006_roll.target_location)); //real和target位置互换最终输出的电流的方向才正确
	motor_velocity_change(&M2006_roll,&hcan1,pid_control3, M2006_roll.target_v);
	
	Can_Fun.CAN_SendData(CAN_SendHandle,&hcan1,CAN_ID_STD,0x1ff,CAN1_0x1ff_Tx_Data);
}
