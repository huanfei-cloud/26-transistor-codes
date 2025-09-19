/**
 * @file chassis.c
 * @author lyd
 * @brief 完成底盘控制任务
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "chassis.h"


struct Struct_CHASSIS_Manage_Object chassis_control = {0};
uint8_t chassis_lock_flag = 0;
uint8_t climb_flag = 0; // flag==1开始爬升
uint8_t last_climb_flag = 0;
float up_limit[4] = {-350000, 350000, 330000, -330000};
float down_limit[4] = {30000, -30000, 0, 0};
float target_pos[4] ={0};

/**
 * @brief 发射处理初始化
 * @param 结构体地址
 */
void chassis_init(struct Struct_CHASSIS_Manage_Object *chassis)
{

  motor_init(&chassis->motor1, 0x201);
  motor_init(&chassis->motor2, 0x202);
  motor_init(&chassis->motor3, 0x203);
  motor_init(&chassis->motor4, 0x204);
  motor_init(&chassis->joint_motor1, 0x205);
  motor_init(&chassis->joint_motor2, 0x206);
  motor_init(&chassis->joint_motor3, 0x207);
  motor_init(&chassis->joint_motor4, 0x208);

  PID_Init(&(chassis->motor1.v_pid_object), 8, 0.03, 6, 1, 10000, 16000, 20);
  PID_Init(&(chassis->motor2.v_pid_object), 8, 0.03, 6, 1, 10000, 16000, 20);
  PID_Init(&(chassis->motor4.v_pid_object), 8, 0.03, 6, 1, 10000, 16000, 20);
  PID_Init(&(chassis->motor3.v_pid_object), 8, 0.03, 6, 1, 10000, 16000, 20);
  PID_Init(&(chassis->joint_motor1.v_pid_object), 17, 0.03, 12, 0, 10000, 30000, 0);
  PID_Init(&(chassis->joint_motor2.v_pid_object), 17, 0.03, 12, 0, 10000, 30000, 0);
  PID_Init(&(chassis->joint_motor3.v_pid_object), 13, 0.03, 9, 0, 10000, 30000, 0);
  PID_Init(&(chassis->joint_motor4.v_pid_object), 13, 0.03, 9, 0, 10000, 30000, 0);
  PID_Init(&(chassis->joint_motor1.l_pid_object), 0.2, 0.001, 0.4, 0.5, 100, 630, 20); //535
  PID_Init(&(chassis->joint_motor2.l_pid_object), 0.2, 0.001, 0.4, 0.5, 100, 630, 20);
  PID_Init(&(chassis->joint_motor3.l_pid_object), 0.2, 0.001, 0.4, 0.5, 100, 400, 20);
  PID_Init(&(chassis->joint_motor4.l_pid_object), 0.2, 0.001, 0.4, 0.5, 100, 400, 20);

  chassis->motor1.target_v = 0.0f;
  chassis->motor2.target_v = 0.0f;
  chassis->motor4.target_v = 0.0f;
  chassis->motor3.target_v = 0.0f;

  chassis->joint_motor1.target_location = encoder_to_quanshu(down_limit[0]);
  chassis->joint_motor2.target_location = encoder_to_quanshu(down_limit[1]);
	chassis->joint_motor3.target_location = encoder_to_quanshu(down_limit[2]);
  chassis->joint_motor4.target_location = encoder_to_quanshu(down_limit[3]);

  for (int i = 0; i < 4; i++)
  {
    chassis->chassis_motor[i].s = s_chassis;
    chassis->chassis_motor[i].r = r_chassis;
  }

  chassis->max_speed = 300;
}

/**
 * @brief  将云台坐标转换为底盘坐标
 * @param  angle 云台相对于底盘的角度
 * @retval 偏差角，弧度制
 * @attention 假定给定速度是以云台为坐标系，此函数将给定速度转化到底盘坐标系下
 */
void Chassis_Absolute_Cal(struct Struct_CHASSIS_Manage_Object *chassis, fp32 angle)
{
  chassis->Speed_ToChassis.w =  3*chassis->Speed_ToCloud.w;
	chassis->Speed_ToChassis.vx = -3 * chassis->Speed_ToCloud.vx * cos(angle) +
                                3 * chassis->Speed_ToCloud.vy * sin(angle);
  chassis->Speed_ToChassis.vy = -3 * chassis->Speed_ToCloud.vx * sin(-angle) +
                                3 * chassis->Speed_ToCloud.vy * cos(-angle);

  // 保证底盘是相对摄像头做移动，当摄像头转过90度时x方向速度从1变0，
  // y方向速度从0变1，保证视觉上是相对右移
}

/**
 * @brief 底盘速度解算
 * @param 结构体地址
 */
void backward_calc(struct Struct_CHASSIS_Manage_Object *chassis)
{
  chassis->chassis_motor[0].omega = (chassis->Speed_ToChassis.vx - chassis->Speed_ToChassis.vy + chassis->Speed_ToChassis.w * chassis->chassis_motor[0].r * sqrt(2)) / chassis->chassis_motor[0].s;
  chassis->chassis_motor[1].omega = (-chassis->Speed_ToChassis.vx - chassis->Speed_ToChassis.vy + chassis->Speed_ToChassis.w * chassis->chassis_motor[1].r * sqrt(2)) / chassis->chassis_motor[1].s;
  chassis->chassis_motor[2].omega = (-chassis->Speed_ToChassis.vx + chassis->Speed_ToChassis.vy + chassis->Speed_ToChassis.w * chassis->chassis_motor[2].r * sqrt(2)) / chassis->chassis_motor[2].s;
  chassis->chassis_motor[3].omega = (chassis->Speed_ToChassis.vx + chassis->Speed_ToChassis.vy + chassis->Speed_ToChassis.w * chassis->chassis_motor[3].r * sqrt(2)) / chassis->chassis_motor[3].s;
}

/**
 * @brief 更改底盘每个电机的目标速度
 * @param 结构体地址
 */
void chassis_task(struct Struct_CHASSIS_Manage_Object *chassis)
{
	float yaw_angle = (ControlMes.yaw_position - 290)/4096.0f*pi;
  Chassis_Absolute_Cal(chassis, yaw_angle); // 绝对坐标系到相对坐标系解算

  backward_calc(chassis);                         // 解算到各轮子
  chassis->motor1.target_v = chassis->chassis_motor[0].omega;
  chassis->motor2.target_v = chassis->chassis_motor[1].omega;
  chassis->motor3.target_v = chassis->chassis_motor[2].omega;
  chassis->motor4.target_v = chassis->chassis_motor[3].omega;
	
	//以下为半自动爬升，通过遥控器控制
  switch (climb_flag)		
  {
	case 0:
	{
		chassis->joint_motor1.target_location = 0;
    chassis->joint_motor2.target_location = 0;
		chassis->joint_motor3.target_location = encoder_to_quanshu(down_limit[2]);
    chassis->joint_motor4.target_location = encoder_to_quanshu(down_limit[3]);
		break;
	}
	
  case 1:
  {
    // 4轮抬起
    // climb_200();
    chassis_control.joint_motor1.target_location = encoder_to_quanshu(up_limit[0]);
    chassis_control.joint_motor2.target_location = encoder_to_quanshu(up_limit[1]);
		chassis_control.joint_motor3.target_location = encoder_to_quanshu(up_limit[2]);
    chassis_control.joint_motor4.target_location = encoder_to_quanshu(up_limit[3]);
		break;
  }
  
  case 2:
  {
    // 回到初始状态
    chassis->joint_motor1.target_location = encoder_to_quanshu(down_limit[0]);
    chassis->joint_motor2.target_location = encoder_to_quanshu(down_limit[1]);
		chassis->joint_motor3.target_location = encoder_to_quanshu(down_limit[2]);
    chassis->joint_motor4.target_location = encoder_to_quanshu(down_limit[3]);
		break;
  }
  
  case 3:
  {
    // 前轮放下
    chassis->joint_motor1.target_location = encoder_to_quanshu(down_limit[0]);
    chassis->joint_motor2.target_location = encoder_to_quanshu(down_limit[1]);
		break;
  }
  
  case 4:
  {
    // 后轮放下
    chassis->joint_motor3.target_location = encoder_to_quanshu(down_limit[2]);
    chassis->joint_motor4.target_location = encoder_to_quanshu(down_limit[3]);
		break;
  }
  
  }
	

  motor_location_change(&chassis_control.joint_motor1, pid_control3, chassis_control.joint_motor1.target_location);
  motor_location_change(&chassis_control.joint_motor2, pid_control3, chassis_control.joint_motor2.target_location);
  motor_location_change(&chassis_control.joint_motor3, pid_control3, chassis_control.joint_motor3.target_location);
  motor_location_change(&chassis_control.joint_motor4, pid_control3, chassis_control.joint_motor4.target_location);

  motor_velocity_change(&chassis->motor1, pid_control3, chassis->motor1.target_v);
  motor_velocity_change(&chassis->motor2, pid_control3, chassis->motor2.target_v);
  motor_velocity_change(&chassis->motor3, pid_control3, chassis->motor3.target_v);
  motor_velocity_change(&chassis->motor4, pid_control3, chassis->motor4.target_v);

  motor_velocity_change(&chassis->joint_motor1, pid_control3, chassis->joint_motor1.target_v);
  motor_velocity_change(&chassis->joint_motor2, pid_control3, chassis->joint_motor2.target_v);
  motor_velocity_change(&chassis->joint_motor3, pid_control3, chassis->joint_motor3.target_v);
  motor_velocity_change(&chassis->joint_motor4, pid_control3, chassis->joint_motor4.target_v);

  fdcanx_send_data(&hfdcan1, 0x200, CAN_0x200_Tx_Data, 8);
  //fdcanx_send_data(&hfdcan2, 0x1ff, CAN_0x1ff_Tx_Data, 8);
}
