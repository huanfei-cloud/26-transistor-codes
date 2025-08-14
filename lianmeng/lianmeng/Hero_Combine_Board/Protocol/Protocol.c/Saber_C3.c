/**
 * @file Saber_C3.c
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-8-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#include "Saber_C3.h"

#include "usart.h"
#include "M6020_Motor.h"
#include "Mecanum_Chassis.h"

void Saber_Init(void);
static float R4(uint8_t *p);
uint8_t Saber_Check(uint8_t *p);
void Saber_Read(void);
void Saber_Online_Check(void);
 

Saber_Angle_t Saber_Angle;
One_Kalman_t Saber_Kalman;
uint8_t Saber_Rxbuffer[Saber_Data_Length];
uint8_t Saber_Montage[Saber_Data_Buffer];

Saber_Fun_t Saber_Fun = Saber_FunGroundInit;
#undef Saber_FunGroundInit
 
 /**
  * @brief ʮ������תfloat
  */
static float R4(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}
 /**
  * @brief          IMU��ʼ�����񣬴򿪴���
  * @retval         none
  */
 void Saber_Init(void)
 {
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Saber_Rxbuffer,sizeof(Saber_Rxbuffer));
 }
 
/**
  * @brief          ֡ͷ֡β�ж�
  * @parameter[in]  ���������׵�ַ
  * @retval         ������Э�鷵��1�����򷵻�0��
  */
uint8_t Saber_Check(uint8_t *p)
{
	/* Header���ڱ����������飬Last���ڱ�Ǳ����Ŀ�ʼ�Լ�����λ�� */
	static uint8_t Header_Position = 0;
	static uint8_t Last_Position;
	Last_Position = Header_Position;
	do
	{
		if(p[Header_Position] == 0x41 && p[(Header_Position+1) % Saber_Data_Buffer] == 0x78 \
			&& p[(Header_Position+2) % Saber_Data_Buffer] == 0xFF && p[(Header_Position+3) \
			% Saber_Data_Buffer]== 0x06 && p[(Header_Position + Frame_End) % Saber_Data_Buffer] == 0X6D)
		{
			return Header_Position;
		}
		Header_Position++;
		if(Header_Position >= Saber_Data_Buffer)
			Header_Position = 0;
	}while(Header_Position!=Last_Position);
	
	return Saber_Data_Buffer; //�ж�ʱ�Ϳ�����ֵ�ǲ���С��Saber_Data_Buffer�ͺ�
}
 

 /**
  * @brief          IMUŷ���Ƕ�ȡ����
  * @retval         none
  */
void Saber_Read(void)
{
	static uint8_t Header_Position;
	Header_Position = Saber_Fun.Saber_Check(Saber_Montage);
	if(Header_Position < Saber_Data_Buffer)             //�ų��쳣ֵ
	{
		Saber_Angle.X_Acc = R4(&Saber_Rxbuffer[Header_Position + 9])*9.801f-0.06f;
		Saber_Angle.Y_Acc = R4(&Saber_Rxbuffer[Header_Position + 13])*9.801f+0.16f;
		Saber_Angle.Z_Acc = R4(&Saber_Rxbuffer[Header_Position + 17])*9.801f;
		
		Saber_Angle.X_Vel = R4(&Saber_Rxbuffer[Header_Position + 24]);
		Saber_Angle.Y_Vel = R4(&Saber_Rxbuffer[Header_Position + 28]);
		Saber_Angle.Z_Vel = R4(&Saber_Rxbuffer[Header_Position + 32]);
		
		
		Saber_Angle.RoLL = R4(&Saber_Rxbuffer[Header_Position + 58]);
		Saber_Angle.Pitch = R4(&Saber_Rxbuffer[Header_Position +  62]);
		Saber_Angle.Yaw  = R4(&Saber_Rxbuffer[Header_Position + 66]);
		
		Saber_Angle.InfoUpdateFrame++;
		Saber_Angle.InfoUpdateFlag = 1;
	}
}

 /**
  * @brief          IMU���߼��
  * @retval         OffLineFlag 1Ϊ���� 0Ϊ����
  */
void Saber_Online_Check(void)
{
	if(Saber_Angle.InfoUpdateFrame < 1)
	{
		Saber_Angle.OffLineFlag = 1;
	}
	else
	{
		Saber_Angle.OffLineFlag = 0;
	}
	Saber_Angle.InfoUpdateFrame = 0;
}
