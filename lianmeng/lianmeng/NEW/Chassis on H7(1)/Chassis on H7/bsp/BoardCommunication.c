/**
 * @file BoardCommunication.c
 * @author lxr(784457420@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 * @brief ControlMes需要发送的信息包括：pitch角度,摩擦轮
 * @copyright Copyright (c) 2023
 *
 */
#include "BoardCommunication.h"
#include "queue.h"

ControlMessge ControlMes;

void Board2_To_1(void);
void Board2_getChassisInfo(uint8_t *RxMessage);
void Board2_getGimbalInfo(uint8_t *RxMessage);
void Board2_getChassisFunctionInfo(uint8_t *RxMessage);

float yaw_velocity = 0;
Board2_FUN_t Board2_FUN = Board2_FunGroundInit;

//此函数用来按照报文规则生成数据并发送。
void Board2_To_1(void)
{
    int16_t bullet_speed;
    uint8_t data1[8] = {0};
    uint8_t data2[8] = {0};

    bullet_speed = (int16_t)(g_referee.shoot_data_.initial_speed * 1000);
    float yaw_angle = AHRSData_Packet.Yaw  / pi * 4096;
    float Gyro_z = AHRSData_Packet.YawSpeed / pi * 180;
    uint8_t game_start=g_referee.game_status_.game_progress;

    data1[0] = (int16_t)bullet_speed >> 8;
    data1[1] = (int16_t)bullet_speed;
    data1[2] = (int16_t)g_referee.robot_status_.shooter_barrel_heat_limit>>8;
    data1[3] = (int16_t)g_referee.robot_status_.shooter_barrel_heat_limit;
    data1[4] = (int16_t)g_referee.power_heat_.shooter_42mm_barrel_heat>>8;
    data1[5] = (int16_t)g_referee.power_heat_.shooter_42mm_barrel_heat;

    data2[0] = (int16_t)yaw_angle >> 8;
    data2[1] = (int16_t)yaw_angle;
    data2[2] = (int16_t)Gyro_z>>8;
    data2[3] = (int16_t)Gyro_z;
    data2[4] = ControlMes.tnndcolor;
    data2[5] = game_start;

    //数据发送
    fdcanx_send_data(&hfdcan2,  CAN_ID_SHOOT_HEAT,data1,8);
    fdcanx_send_data(&hfdcan2,CAN_ID_SABER,data2, 8);

}

//此函数用来解析CAN数据，同时将结果直接赋值给底盘
void Board2_getChassisInfo(uint8_t *RxMessage)
{
    float vx = (int16_t)(RxMessage[0] << 8 | RxMessage[1]);
    float vy = (int16_t)(RxMessage[2] << 8 | RxMessage[3]);
    float vw = (int16_t)(RxMessage[4] << 8 | RxMessage[5]);
    ControlMes.yaw_position = (int16_t)(RxMessage[6] << 8 | RxMessage[7]);
    //注意cloud角度还未更新，后续需要加上

    chassis_control.Speed_ToCloud.vx = vx; //左手上下
    chassis_control.Speed_ToCloud.vy = vy; //左手左右
    chassis_control.Speed_ToCloud.w = -vw;//滑轮

}

void Board2_getGimbalInfo(uint8_t *RxMessage)
{
    static float AutoAim_Offset = 0;
    //ControlMes.yaw_position = (int16_t)(RxMessage[1] << 8 | RxMessage[2]);
    ControlMes.shoot_Speed 	= (uint8_t)RxMessage[3];
    ControlMes.fric_Flag 		= (uint8_t)RxMessage[4];
    ControlMes.AutoAimFlag 	= (uint8_t)RxMessage[5] ;
    ControlMes.change_Flag 	= (uint8_t)RxMessage[6];
    ControlMes.modelFlag 		= (uint8_t)RxMessage[7];

    if (ControlMes.AutoAimFlag)
    {
        AutoAim_Offset += -1 * yaw_velocity * 0.006f; // 右手左右
        //yaw_control.Target_Yaw = yaw_position + AutoAim_Offset; // 此处的值应与上位机传来的数据相同
    }
    else
        AutoAim_Offset = 0;

}

void Board2_getChassisFunctionInfo(uint8_t *RxMessage)
{
    climb_flag= RxMessage[0];
    //deploy_flag=RxMessage[3];
}
