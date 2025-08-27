/**
 * @file Arm_Control.c
 * @author Transistor
 * @brief 机械臂的控制程序
 * @version 1.2
 * @date 2025-08-15
 * @copyright Transistor BUAA
 */

#include "Arm_Control.h"

/********变量定义********/
//机械臂电机结构体定义
#define joint_motor1 motor[Motor1];             // 4310 yaw轴
#define joint_motor2 motor[Motor2];             // 8009 小臂
#define joint_motor3 motor[Motor5];             // 8009 大臂
struct Struct_MOTOR_Manage_Object joint_motor4; // 6020
struct Struct_MOTOR_Manage_Object joint_motor5; // 2006
struct Struct_MOTOR_Manage_Object joint_motor6; // 2006
//数据定义
float arm_angle[6] = {0};
float last_arm_angle[6] = {0};
float target[6] = {0};

//static float round_to_one_decimal(float num)
//{
//    return round(num * 10) / 10; // 四舍五入到小数点后一位
//}

// pos为绝对目标角度度数
void dm_ctrl(motor_t *motor, float pos)
{
    mit_ctrl(&hfdcan2, motor, motor->id, pos, 0, motor->ctrl.kp_set, motor->ctrl.kd_set, 0);
}

/**
 * @brief 机械臂电机初始化并使能
 * @param None
 * @retval None
 */
void arm_init(void)
{
    dm_motor_init();

    dm_motor_enable(&hfdcan2, &motor[Motor1]);
    dm_motor_enable(&hfdcan2, &motor[Motor2]);
    dm_motor_enable(&hfdcan2, &motor[Motor5]);

    motor_init(&joint_motor4, 0x208);
    motor_init(&joint_motor5, 0x202);
    motor_init(&joint_motor6, 0x203);

    PID_Init(&(joint_motor4.v_pid_object), 5, 0.03, 0.3, 0, 10000, 16000, 200);
    PID_Init(&(joint_motor5.v_pid_object), 5, 0, 5, 0, 10000, 30000, 20);
    PID_Init(&(joint_motor6.v_pid_object), 5, 0,5, 0, 10000, 30000, 20);

    PID_Init(&(joint_motor4.l_pid_object), 5, 0.03, 0.3, 0, 10000, 16000, 20);
    PID_Init(&(joint_motor5.l_pid_object), 0.5, 0, 0, 0, 1000, 16000, 20);
    PID_Init(&(joint_motor6.l_pid_object), 0.5, 0, 0, 0, 1000, 16000, 20);

    arm_angle[0] = 0;
    arm_angle[1] = 0;
    arm_angle[2] = 0;
    arm_angle[3] = 0;
    arm_angle[4] = 0;
    arm_angle[5] = 0;

    joint_motor4.target_location = 0.0f;
    joint_motor5.target_location = 0.0f;
    joint_motor6.target_location = 0.0f;


    //    dm_motor_disable(&hfdcan2, &motor[Motor1]);
    //    dm_motor_disable(&hfdcan2, &motor[Motor2]);
    //    dm_motor_disable(&hfdcan2, &motor[Motor3]);


}

/**
 * @brief 机械臂达妙电机控制
 * @param None
 * @retval None
 */
void DM_Motors_Out(void)
{
	if (motor[Motor1].zero_flag == 1 && motor[Motor2].zero_flag == 1 && motor[Motor5].zero_flag == 1)
        {

//            target[0] = arm_angle[0] / 180.0f * pi + motor[Motor1].zero;
//            target[1] = arm_angle[1] / 180.0f * pi + motor[Motor2].zero;
//            target[2] = arm_angle[2] / 180.0f * pi + motor[Motor5].zero;
            dm_ctrl(&motor[Motor5], motor[Motor5].zero - 2*(arm_angle[1] / 180.0f * pi));
            dm_ctrl(&motor[Motor1], motor[Motor1].zero - arm_angle[0] / 180.0f * pi); // 单位为弧度
            dm_ctrl(&motor[Motor2], motor[Motor2].zero - ((arm_angle[2] / 180.0f * pi - arm_angle[1] / 180.0f * pi) * 1.6f));
            //			dm_ctrl(&motor[Motor1],  motor[Motor1].zero); // 单位为弧度
            //			dm_ctrl(&motor[Motor2],  motor[Motor2].zero);
            //			dm_ctrl(&motor[Motor3],  motor[Motor3].zero);
        }
}

/**
 * @brief 机械臂大疆电机控制
 * @param None
 * @retval None
 */
void DJ_Motors_Out(void)
{
	joint_motor4.target_location = arm_angle[3] / 360.0f; // 相对于上电零点的圈数
        joint_motor5.target_location = -36*(arm_angle[4]/360.0f );
        joint_motor6.target_location = -36*(arm_angle[5]/360.0f );
//				joint_motor5.target_location = 0;
//        joint_motor6.target_location = 0;

        motor_location_change(&joint_motor4, pid_control2, joint_motor4.target_location);
        motor_location_change(&joint_motor5, pid_control2, joint_motor5.target_location);
        motor_location_change(&joint_motor6, pid_control2, joint_motor6.target_location);
        motor_velocity_change(&joint_motor4, pid_control2, joint_motor4.target_v);
        motor_velocity_change(&joint_motor5, pid_control2, joint_motor5.target_v);
        motor_velocity_change(&joint_motor6, pid_control2, joint_motor6.target_v);


        fdcanx_send_data(&hfdcan1, 0x200, CAN_0x200_Tx_Data, 8);
        fdcanx_send_data(&hfdcan1, 0x1ff, CAN_0x1ff_Tx_Data, 8);
}
