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
 * @version 0.2
 * @date 2024-1-28
 *
 */
#include "Protocol_UpperComputer.h"
#include "BSP_BoardCommunication.h"
#include "FrictionWheel.h"
#include "Cloud_Control.h"
#include "usbd_cdc_if.h"
#include "M6020_Motor.h"

float Auto_Aim_Yaw;
float Auto_Aim_Pitch;
positionpid_t Auto_Aim_PID;

// 测试debug显示用的变量
float AutoYaw;
float AutoPitch;
float abs_pitch;
uint16_t Timeout = 1000;
/**
  * @brief  UpperCom下位机与上位机通信，向上位机发送信息。使用USB
  * @param  void
  * @retval void
  */
void UpperCom_Send_To_Up(uint8_t COM)
{
	uint8_t UpperCom_Send_Buffer[UpperCom_MAX_BUF];

	/*获取弹速*/
	float bullet_velocity = 24.0f;
	if (ControlMes.Speed_Bullet >= 5)
		bullet_velocity = ControlMes.Speed_Bullet;
	float bullet_angle = 0;//(M6020s_Pitch.realAngle - Cloud_Pitch_level) / 22.75556;      //Pitch轴的水平夹角（水平线以上为负）
	int16_t gimbal_yaw = ControlMes.yaw_realAngle/22.75556;                            //底盘传上来的Yaw轴角度
	static uint16_t mark = 0;                                                          //时间戳（没什么用）
	
	/* 先录入帧头 */
	UpperCom_Send_Buffer[0] = 0x42;
	UpperCom_Send_Buffer[1] = 0x52;
	UpperCom_Send_Buffer[2] = COM;
	/* 再根据命令码加上数据帧和CRC8的校验 */
	if(COM == 0xCD)
	{
		if(mark++ >= 200) mark = 0;
		bullet_velocity+=1;
		UpperCom_Send_Buffer[3] = 15;//数据包包含的字节数
		memcpy(&UpperCom_Send_Buffer[4],&bullet_velocity,sizeof(bullet_velocity));
		memcpy(&UpperCom_Send_Buffer[8],&bullet_angle,sizeof(bullet_angle));
		memcpy(&UpperCom_Send_Buffer[12],&gimbal_yaw,sizeof(gimbal_yaw));
		memcpy(&UpperCom_Send_Buffer[14],&mark,sizeof(mark));
		memcpy(&UpperCom_Send_Buffer[16],&ControlMes.tnndcolor,sizeof(ControlMes.tnndcolor));           //红方/蓝方
		memcpy(&UpperCom_Send_Buffer[17],&ControlMes.z_rotation_velocity,sizeof(ControlMes.z_rotation_velocity));     

		Append_CRC8_Check_Sum(UpperCom_Send_Buffer, 5+UpperCom_Send_Buffer[3]);//5+x，x代表数据包包含的数据字节数。
	}
	CDC_Transmit_FS(UpperCom_Send_Buffer,sizeof(UpperCom_Send_Buffer));//usb发送
	memset(UpperCom_Send_Buffer, 0, UpperCom_MAX_BUF);

}

 /**
  * @brief 十六进制转float
  */
static float R2float(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}

 /**
  * @brief 十六进制转int16_t
  */
static float R2int16(uint8_t *p) {int16_t r; memcpy(&r, p, 2); return r;}


void UpperCom_Receive_From_Up(uint8_t Rec[])
{	
  abs_pitch = Auto_Aim_Pitch + M6020s_Pitch.realAngle;
	/* 先检验帧头 */
	if(Rec[0] != 0x42 ||Rec[1] != 0x52)
		return;
	/* 再根据命令码CRC校验 */
	switch(Rec[2])
	{
		case 0xFF:
		case 0x00:
			Auto_Aim_Pitch = M6020s_Pitch.realAngle;
			Auto_Aim_Yaw = ControlMes.yaw_realAngle;
			break;
		case 0xCD:
			if(!Verify_CRC8_Check_Sum(Rec, 5+Rec[3])) return;   //接收异常
			AutoPitch = R2float(&Rec[4]);          //接收原始数据
			AutoYaw = R2int16(&Rec[8]);            //接收原始数据		
		
		  AutoYaw /=100;         								 //上位机下发的数据是100倍
		  Auto_Aim_Yaw = AutoYaw * Test_Yaw_SEN; //将原始数据映射到电机编码值（8192）		
		  Auto_Aim_Pitch = AutoPitch * Test_Pitch_SEN;       //将原始数据映射到电机编码值（8192）

			break;
		default:
			return;
	}
	
	
}
