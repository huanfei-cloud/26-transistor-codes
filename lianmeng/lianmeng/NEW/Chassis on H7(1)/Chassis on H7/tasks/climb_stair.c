#include "climb_stair.h"

float climb_time = 0;

// extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	climb_time+=0.005f;
// }

void climb_200(void)
{
	if (last_climb_flag == 0)
	{
		climb_time = 0;
		//HAL_TIM_Base_Start_IT(&htim3);
		last_climb_flag = 1;
	}

	if (climb_time < 2000) // 10s内完成四轮抬升
	{
		chassis_control.joint_motor1.target_location = encoder_to_quanshu(up_limit[0]);
		chassis_control.joint_motor2.target_location = encoder_to_quanshu(up_limit[1]);
		chassis_control.joint_motor3.target_location = encoder_to_quanshu(up_limit[2]);
		chassis_control.joint_motor4.target_location = encoder_to_quanshu(up_limit[3]);
	}
	else if (climb_time < 3000) // 5s后
	{
		chassis_control.motor3.target_location = 25;
		chassis_control.motor4.target_location = 25;

		// 后轮前进，大概是两个前轮的距离，需要测定
	}
	else if (climb_time < 15)
	{
		chassis_control.joint_motor1.target_location = 0.0f;
		chassis_control.joint_motor2.target_location = 0.0f;
		// 前轮支撑杆收起
	}
	else if (climb_time < 20)
	{
		chassis_control.motor1.target_location = 20;
		chassis_control.motor2.target_location = 20;
		// 前轮前进
	}
	else if (climb_time < 25)
	{
		chassis_control.joint_motor1.target_location = 1;
		chassis_control.joint_motor2.target_location = -1;
		// 后轮支撑杆收起
	}
	else if (climb_time < 30)
	{
		chassis_control.motor1.target_location = 20;
		chassis_control.motor2.target_location = 20;

		// 前轮前进，完全上台阶
	}
	else
	{
		climb_flag = 0;
		last_climb_flag = 0; // 进入普通模式
		//HAL_TIM_Base_Stop_IT(&htim3);
	}

	motor_location_change(&chassis_control.motor1, pid_control3, chassis_control.motor1.target_location);
	motor_location_change(&chassis_control.motor2, pid_control3, chassis_control.motor2.target_location);
	motor_location_change(&chassis_control.motor3, pid_control3, chassis_control.motor3.target_location);
	motor_location_change(&chassis_control.motor4, pid_control3, chassis_control.motor4.target_location);
}
