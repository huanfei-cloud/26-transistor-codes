/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @frame 帧头 0x42 0x52  —— ASCII是BR
 *  	  对于弹道数据，第三个字节是命令码,0xCD；
 *  	  第四个字节是数据帧的长度；
 *  	  之后是数据帧，弹道有关的有一个字节
 *		  下位机发给上位机是射速，上位机发给下位机是云台100倍的俯仰*弧度*；
 *	  	  帧尾 一字节的CRC8
 *
 * @version 0.2
 * @date 2024-1-28
 *
 */
#include "Protocol_UpperComputer.h"
#include "BSP_BoardCommunication.h"
#include "FrictionWheel.h"
#include "Cloud_Control.h"
#include "usbd_cdc_if.h"
#include "BSP_BoardCommunication.h"

float Auto_Aim_Yaw;
float Auto_Aim_Pitch;
positionpid_t Auto_Aim_PID;
float Test_Yaw_Sen = 1;

// 测试debug显示用的变量
float AutoYaw;
float AutoPitch;
float test;
int kk = 1;
float test_pitch_sen = 0;

/**
 * @brief  UpperCom下位机与上位机通信，向上位机发送信息。
 * @param  void
 * @retval void
 */
void UpperCom_Send_To_Up(uint8_t COM)
{
	uint8_t UpperCom_Send_Buffer[UpperCom_MAX_BUF];

	/*获取弹速*/
	float bullet_velocity = 12.0f;
	//	if (ControlMes.Speed_Bullet >= 5)
	//		bullet_velocity = ControlMes.Speed_Bullet;
	float bullet_angle = -(Cloud_Pitch_level - J4310s_Pitch.realAngle);
	int16_t gimbal_yaw = Aim_Yaw_RawAngle * 4.39453125; //   /8192*360
	static uint16_t mark = 0;
	// ControlMes.z_rotation_velocity = 0;
	/* 先录入帧头 */
	UpperCom_Send_Buffer[0] = 0x42;
	UpperCom_Send_Buffer[1] = 0x52;
	UpperCom_Send_Buffer[2] = COM;
	/* 再根据命令码加上数据帧和CRC8的校验 */
	if (COM == 0xCD)
	{
		if (mark++ >= 200)
			mark = 0;
		UpperCom_Send_Buffer[3] = 15; // 数据包包含的字节数
		memcpy(&UpperCom_Send_Buffer[4], &bullet_velocity, sizeof(bullet_velocity));
		memcpy(&UpperCom_Send_Buffer[8], &bullet_angle, sizeof(bullet_angle));
		memcpy(&UpperCom_Send_Buffer[12], &gimbal_yaw, sizeof(gimbal_yaw));
		memcpy(&UpperCom_Send_Buffer[14], &mark, sizeof(mark));
		memcpy(&UpperCom_Send_Buffer[16], &ControlMes.tnndcolor, sizeof(ControlMes.tnndcolor));
		memcpy(&UpperCom_Send_Buffer[17], &ControlMes.z_rotation_velocity, sizeof(ControlMes.z_rotation_velocity));
		Append_CRC8_Check_Sum(UpperCom_Send_Buffer, 5 + UpperCom_Send_Buffer[3]); // 5+x，x代表数据包包含的数据字节数。
	}
	CDC_Transmit_FS(UpperCom_Send_Buffer, sizeof(UpperCom_Send_Buffer)); // usb发送
	memset(UpperCom_Send_Buffer, 0, UpperCom_MAX_BUF);
}

/**
 * @brief 十六进制转float
 */
static float R2float(uint8_t *p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}

/**
 * @brief 十六进制转int16_t
 */
static float R2int16(uint8_t *p)
{
	int16_t r;
	memcpy(&r, p, 2);
	return r;
}

/**
 * @brief  UpperCom下位机与上位机通信，接受上位机的信息。使用usart*
 * @param  *Rec 接收到的一帧数据
 * @retval void
 */
void UpperCom_Receive_From_Up(uint8_t Rec[])
{
	/* 先检验帧头 */
	if (Rec[0] != 0x42 || Rec[1] != 0x52)
		return;
	/* 再根据命令码CRC校验 */
	switch (Rec[2])
	{
		//		case 0xFF:
		//		case 0x00:
		//			Auto_Aim_Pitch = M6020s_Pitch.realAngle;
		//			Auto_Aim_Yaw = ControlMes.yaw_realAngle;
		//			break;
	case 0xCD:
		if (!Verify_CRC8_Check_Sum(Rec, 5 + Rec[3]))
			return;
		//			Auto_Aim_Pitch = Cloud_Pitch_level - R2float(&Rec[4]) * 1303.8f;     // 配合云台控制的跨圈
		AutoPitch = R2float(&Rec[4]);
		Auto_Aim_Pitch = AutoPitch * test_pitch_sen;

		// Auto_Aim_Yaw = Position_PID(&Auto_Aim_PID, 0, R4(&Rec[8]));
		AutoYaw = R2int16(&Rec[8]);
		AutoYaw /= 100;
		Auto_Aim_Yaw = AutoYaw / 360 * 8192 * Test_Yaw_Sen;

		break;
	default:
		return;
	}
}
