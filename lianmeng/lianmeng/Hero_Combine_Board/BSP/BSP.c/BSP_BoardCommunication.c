/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "BSP_BoardCommunication.h"
#include "Protocol_Judgement.h"
#include "Mecanum_Chassis.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"

ControlMessge ControlMes;

void Board2_To_1(void);
void Board2_getChassisInfo(Fdcan_Export_Data_t RxMessage);
void Board2_getGimbalInfo(Fdcan_Export_Data_t RxMessage);
void Board2_getChassisFunctionInfo(Fdcan_Export_Data_t RxMessage);
void Board1_To_2(void);
void Board1_getGimbalInfo(Fdcan_Export_Data_t RxMessage);
void Board1_getSaberInfo(Fdcan_Export_Data_t RxMessage);
void Board1_getShootHeatInfo(Fdcan_Export_Data_t RxMessage);

Board1_FUN_t Board1_FUN = Board1_FunGroundInit;
Board2_FUN_t Board2_FUN = Board2_FunGroundInit;
	uint8_t data[8]={0};
	uint8_t data2[8]={0};


extern M6020s_t *M6020_Array[2];
fp32 YawAngle_Offset = 20/360*8192;         //需测定
IMU_Angle_t IMU_Angle;
 /**
  * @brief 十六进制转float
  */
static float R2float(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}


//此函数用来按照报文规则生成数据并发送。
void Board1_To_2(void)
{
//	uint8_t data2[8]={0};
	
	//底盘移动数据
	data[0] = ControlMes.x_velocity >> 8;
	data[1] = ControlMes.x_velocity ;
	data[2] = ControlMes.y_velocity >> 8;
	data[3] = ControlMes.y_velocity ;
	data[4] = ControlMes.z_rotation_velocity >> 8;
	data[5] = ControlMes.z_rotation_velocity;
	data[6] = M6020_Array[0]->realAngle >> 8;
	data[7] = M6020_Array[0]->realAngle;
	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_CHASSIS, data);
	
//	//底盘功能开关
//	data2[0] = ControlMes.Climb_Whole_Flag;           //爬升开关（整体）
//	data2[1] = ControlMes.Climb_Front_Flag;           //爬升开关（前轮）
//	data2[2] = ControlMes.Climb_Back_Flag;            //爬升开关（后轮）	
//	data2[3] = ControlMes.Deployment_Flag;            //部署模式
//	data2[4] = ControlMes.Check_In_Flag;              //检录开关
//	//数据发送
//	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle,&hfdcan2,FDCAN_STANDARD_ID,FDCAN_ID_GIMBAL,data2);
}

void Board1_getGimbalInfo(Fdcan_Export_Data_t RxMessage)
{
	ControlMes.yaw_realAngle = (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
	Saber_Angle.Yaw = ControlMes.yaw_realAngle;
	ControlMes.Blood_Volume = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
	ControlMes.shooter_42mm_heat_limit = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
	ControlMes.shooter_42mm_heat_now = (int16_t)(RxMessage.FDCANx_Export_RxMessage[6] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
	ControlMes.Speed_Bullet = R2float(&(RxMessage.FDCANx_Export_RxMessage[3]));
	ControlMes.tnndcolor = RxMessage.FDCANx_Export_RxMessage[7];
}

//此函数用来解析CAN数据，同时将结果直接赋值给底盘
void Board2_getChassisInfo(Fdcan_Export_Data_t RxMessage)
{
	float vx = (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
	float vy = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
	float vw = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
//	ControlMes.yaw_velocity =(int16_t)(RxMessage.FDCANx_Export_RxMessage[6] << 8 | RxMessage.FDCANx_Export_RxMessage[7]);
	//注意cloud角度还未更新，后续需要加上
	Mecanum_Data.Speed_ToCloud.vx =  vx*2;
	Mecanum_Data.Speed_ToCloud.vy =  -1*vy*2;
	Mecanum_Data.Speed_ToCloud.vw =  (float)vw/100;
//	if(ControlMes.AutoAimFlag !=1 )
//	Cloud.Target_Yaw+=ControlMes.yaw_velocity* 0.003f * 25;
}

void Board2_getChassisFunctionInfo(Fdcan_Export_Data_t RxMessage)
{
	ControlMes.Climb_Whole_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[0];  //爬升开关（整体）
	ControlMes.Climb_Front_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[1];  //爬升开关（前轮）
	ControlMes.Climb_Back_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[2];  //爬升开关（后轮）
	ControlMes.Deployment_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[3]; //部署模式开关
}

void Board2_getGimbalInfo(Fdcan_Export_Data_t RxMessage)
{
	static float AutoAim_Offset = 0;
	ControlMes.AutoAimFlag =(uint8_t)RxMessage.FDCANx_Export_RxMessage[0];
	float yaw_position = (int16_t)(RxMessage.FDCANx_Export_RxMessage[1] << 8 | RxMessage.FDCANx_Export_RxMessage[2]);
	ControlMes.fric_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[3];
	ControlMes.Check_In_Flag = (uint8_t)RxMessage.FDCANx_Export_RxMessage[4];
//	if(ControlMes.AutoAimFlag  == 1)
//	{
//		AutoAim_Offset += ControlMes.yaw_velocity* 0.03f ;
//		Cloud.Target_Yaw = yaw_position + AutoAim_Offset; // 此处的值应与上位机传来的数据相同
//	}
//	else
//	{
//		AutoAim_Offset = 0;
//	}
	
}
void Board2_To_1(void)
{
//	float bullet_speed;
//    uint8_t data2[8] = {0};

//    // 把yaw的角度传上去
//    data2[0] = 0xFF;
//    data2[1] = (int16_t)Cloud.Target_Yaw >> 8;
//    data2[2] = (int16_t)Cloud.Target_Yaw;
//	bullet_speed = ext_shoot_data.data.bullet_speed;
//    memcpy(&data2[3], &bullet_speed, sizeof(bullet_speed));
//	data2[7] = ControlMes.tnndcolor;
//	
//	//把陀螺仪数据传上去
//	uint8_t data3[8] = {0};
//	memcpy(&data3[0],&Saber_Angle.X_Acc,sizeof(Saber_Angle.X_Acc));
//	memcpy(&data3[4],&Saber_Angle.Y_Acc,sizeof(Saber_Angle.Y_Acc));
//	
//	//把裁判系统和云台与底盘中轴偏差角传上去
//	uint8_t data4[8] = {0};
//	memcpy(&data4[0],&ControlMes.shooter_42mm_heat_limit,sizeof(ControlMes.shooter_42mm_heat_limit));
//	memcpy(&data4[2],&ControlMes.shooter_42mm_heat_now,sizeof(ControlMes.shooter_42mm_heat_now));
//	memcpy(&data4[4],&Mecanum_Data.Angle_ChassisToCloud,sizeof(Mecanum_Data.Angle_ChassisToCloud));

//    //数据发送
//    Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_GIMBAL, data2);
//	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_SABER, data3);
//	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_SHOOT_HEAT, data4);
}

/**
  * @brief 十六进制转float
  */
static float R4(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}

void Board1_getSaberInfo(Fdcan_Export_Data_t RxMessage)
{
	IMU_Angle.Yaw = (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
	if(IMU_Angle.Yaw>-2&&IMU_Angle.Yaw<2)
	{
		IMU_Angle.Yaw = 0;
	}
  IMU_Angle.Z_Vel = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
	ControlMes.tnndcolor = RxMessage.FDCANx_Export_RxMessage[4];
//	if (ControlMes.tnndcolor==1)ControlMes.tnndcolor=0;
//	else if(ControlMes.tnndcolor==0)ControlMes.tnndcolor=1;
	ControlMes.game_start = RxMessage.FDCANx_Export_RxMessage[5];
}


void Board1_getShootHeatInfo(Fdcan_Export_Data_t RxMessage)
{
	uint8_t speed[2];
	speed[0] = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
	ControlMes.Speed_Bullet = R2float(&speed[0]);
	ControlMes.shooter_42mm_heat_limit = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
	ControlMes.shooter_42mm_heat_now =(uint16_t) (RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
	ControlMes.Blood_Volume = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[6] << 8 | RxMessage.FDCANx_Export_RxMessage[7]);
}
