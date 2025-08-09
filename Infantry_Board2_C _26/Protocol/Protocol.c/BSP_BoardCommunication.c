/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)/ZS(2729511164@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "BSP_BoardCommunication.h"

ControlMessge ControlMes;
void Board2_To_1(void);
void Board2_getChassisInfo(Can_Export_Data_t RxMessage);
void Board2_getGimbalInfo(Can_Export_Data_t RxMessage);

float yaw_velocity = 0;
Board2_FUN_t Board2_FUN = Board2_FunGroundInit;

extern Saber_Angle_t Saber_Angle;

//�˺����������ձ��Ĺ����������ݲ����͡�
void Board2_To_1(void)
{
	int16_t bullet_speed;
	uint8_t IT_keycommand[8]={0};
	uint8_t data[8] = {0};

	// �ֱ���yaw��Ƕȣ�Saber��pitch��Ƕȣ���һ���ĵ��٣�װ����ɫ
	data[0] = ControlMes.yaw_realAngle >> 8;
	data[1] = ControlMes.yaw_realAngle;	
	data[2] = ControlMes.Blood_Volume >> 8;
	data[3] = ControlMes.Blood_Volume;
	bullet_speed = (int16_t)(g_referee.shoot_data_.initial_speed * 1000);
	data[4] = bullet_speed >> 8;
	data[5] = bullet_speed;
  data[6] |= (uint8_t)(ControlMes.tnndcolor & 0x01) <<0;
	data[6] |= (uint8_t)(ControlMes.game_start & 0x01) <<1;
	
	//ͼ������ӳ�䣬�ֱ������x�ᡢy�ᡢz�ᣨz���ǹ��֣���������Ҽ�����ʱֻ����������Ϣ
	//��ֻ̨��Ҫ���ٿأ�������ϰ����ʱ��Ҫ��չIT_keycommand[8]��IT_keycommand[12]������λֱ�Ӹ�0����
	//����������ң��������ӳ��ĸ�ʽһ��
	//	IT_keycommand[0] = ext_robot_keycommand.data.mouse_x >> 8;
	//	IT_keycommand[1] = ext_robot_keycommand.data.mouse_x;
	//	IT_keycommand[2] = ext_robot_keycommand.data.mouse_y >> 8;
	//	IT_keycommand[3] = ext_robot_keycommand.data.mouse_y;
	//	IT_keycommand[4] = ext_robot_keycommand.data.mouse_z >> 8;
	//	IT_keycommand[5] = ext_robot_keycommand.data.mouse_z;
	//	IT_keycommand[6] = ext_robot_keycommand.data.left_button_down;
	//	IT_keycommand[7] = ext_robot_keycommand.data.right_button_down;
	
  //���ݷ���
  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_GIMBAL, data);
  //Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_KEYCOMMAND, IT_keycommand);
}

/**
  * @brief ����CAN���ݣ�ͬʱ�����ֱ�Ӹ�ֵ������
  * @param RxMessage ���յ�������
  * @retval None
  */
void Board2_getChassisInfo(Can_Export_Data_t RxMessage)
{
    float vx = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    float vy = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    float vw = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    yaw_velocity = (int16_t)(RxMessage.CANx_Export_RxMessage[6] << 8 | RxMessage.CANx_Export_RxMessage[7]);
    //ע��cloud�ǶȻ�δ���£�������Ҫ����

    Omni_Data.Speed_ToCloud.vx = vx; //��������
    Omni_Data.Speed_ToCloud.vy = vy; //��������
    Omni_Data.Speed_ToCloud.vw = -1 * vw / 200; //����
		if(!ControlMes.AutoAimFlag )
  	{
			Cloud.Target_Yaw += -1 * yaw_velocity * 0.06f; // ��������
		}
}

void Board2_getGimbalInfo(Can_Export_Data_t RxMessage)
{
		static float AutoAim_Offset = 0;
    float yaw_position 			= (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
 		ControlMes.shoot_Speed 	= (uint8_t)RxMessage.CANx_Export_RxMessage[2];
	  ControlMes.shoot_Speed/=2;
		ControlMes.fric_Flag 		= (uint8_t)(RxMessage.CANx_Export_RxMessage[3]>>0)&0x01;
		ControlMes.AutoAimFlag 	= (uint8_t)(RxMessage.CANx_Export_RxMessage[3]>>1)&0x01;
		ControlMes.change_Flag 	= (uint8_t)(RxMessage.CANx_Export_RxMessage[3]>>2)&0x01;
		ControlMes.modelFlag 		= (uint8_t)(RxMessage.CANx_Export_RxMessage[3]>>3)&0x01;
		Dial_2006.realTorque    = (uint16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);

		if(ControlMes.AutoAimFlag == 1 )
		{
			if(yaw_position == 0.0f) yaw_position = Cloud.Target_Yaw;

			AutoAim_Offset += -1 * yaw_velocity * 0.05f; // ��������
			Cloud.Target_Yaw = yaw_position + AutoAim_Offset; // �˴���ֵӦ����λ��������������ͬ
		}
		else
		{
			AutoAim_Offset =0;
		}
		

	
}
