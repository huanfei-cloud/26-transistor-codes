/**
 * @file DT7.c
 * @author lxr(784457420@qq.com),cyx(1686143358@qq.com)
 * @brief ���̿���
 * @version 2.0
 * @date 2024-3-9
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "DT7.h"
#include "chassis.h"

/* ----------------------- Internal Data ----------------------------------- */
RC_Ctl_t RC_CtrlData;
static uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];
uint8_t DT7_RX_Finish;
void RemoteControl_PC_Update(void);

DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;

/**
 * @brief  DT7��ʼ�����������տ����ж�
 * @param  void
 * @retval void
 */
void DT7_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, DT7_Rx_Data, sizeof(DT7_Rx_Data));
}

/**
 * @Data    2024/3/9
 * @brief   ���̱�־λ����
 * @param   void
 * @retval  void
 */
void KeyMouseFlag_Update(void)
{
	uint32_t KeyMouse = (uint32_t)RC_CtrlData.key.key_code | RC_CtrlData.mouse.press_l << 16 | RC_CtrlData.mouse.press_r << 17; // �Ѽ������ı�־λ�ϲ���

	for (int Index = 0; Index < KEYMOUSE_AMOUNT; Index++) // ����ȫ����λ���������ǵ�״̬��
	{
		if (KeyMouse & (1 << Index)) // �жϵ�indexλ�Ƿ�Ϊ1��
		{
			DR16_Export_Data.KeyMouse.PressTime[Index]++;
			if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) // ���㰴�µ�ʱ�䣬��Ϊ����
			{
				DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; // ���øü��ı�־λΪ1
			}

			if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) // �����ж�
			{

				DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index; // ���ó�����־λ
			}
		}
		else
		{
			if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) // ʱ�䴦������֮�䣬Ϊ������
			{
				DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; // ���õ�����־λ
			}
			else
			{
				DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); // ȡ�����������ü��ı�־λ��Ϊ0
			}

			// �Ѿ��ɿ��������±�־λ�ÿա�
			DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
			DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
			DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
		}
	}
}

/**
 * @brief  DT7���ݽ���
 * @param  void
 * @retval void
 */
void DT7_Handle(void)
{
	/*DT7���ݴ���*/
	if (DT7_RX_Finish == 1)
	{
		DT7_RX_Finish = 0;

		RC_CtrlData.rc.ch0 = (DT7_Rx_Data[0] | DT7_Rx_Data[1] << 8) & 0x07FF;
		RC_CtrlData.rc.ch0 -= 1024;
		RC_CtrlData.rc.ch1 = (DT7_Rx_Data[1] >> 3 | DT7_Rx_Data[2] << 5) & 0x07FF;
		RC_CtrlData.rc.ch1 -= 1024;
		RC_CtrlData.rc.ch2 = (DT7_Rx_Data[2] >> 6 | DT7_Rx_Data[3] << 2 | DT7_Rx_Data[4] << 10) & 0x07FF;
		RC_CtrlData.rc.ch2 -= 1024;
		RC_CtrlData.rc.ch3 = (DT7_Rx_Data[4] >> 1 | DT7_Rx_Data[5] << 7) & 0x07FF;
		RC_CtrlData.rc.ch3 -= 1024;

		/* prevent remote control zero deviation */
		if (RC_CtrlData.rc.ch0 <= 150 && RC_CtrlData.rc.ch0 >= -150)
		{
			RC_CtrlData.rc.ch0 = 0;
		}
		if (RC_CtrlData.rc.ch1 <= 150 && RC_CtrlData.rc.ch1 >= -150)
		{
			RC_CtrlData.rc.ch1 = 0;
		}
		if (RC_CtrlData.rc.ch2 <= 150 && RC_CtrlData.rc.ch2 >= -150)
		{
			RC_CtrlData.rc.ch2 = 0;
		}
		if (RC_CtrlData.rc.ch3 <= 150 && RC_CtrlData.rc.ch3 >= -150)
		{
			RC_CtrlData.rc.ch3 = 0;
		}

		RC_CtrlData.rc.s1 = ((DT7_Rx_Data[5] >> 4) & 0x000C) >> 2;
		RC_CtrlData.rc.s2 = (DT7_Rx_Data[5] >> 4) & 0x0003;

		RC_CtrlData.mouse.x = DT7_Rx_Data[6] | (DT7_Rx_Data[7] << 8); // x axis
		RC_CtrlData.mouse.y = DT7_Rx_Data[8] | (DT7_Rx_Data[9] << 8);
		RC_CtrlData.mouse.z = DT7_Rx_Data[10] | (DT7_Rx_Data[11] << 8);

		RC_CtrlData.mouse.press_l = DT7_Rx_Data[12];
		RC_CtrlData.mouse.press_r = DT7_Rx_Data[13];

		RC_CtrlData.key.key_code = DT7_Rx_Data[14] | DT7_Rx_Data[15] << 8; // key borad code
		RC_CtrlData.wheel = (DT7_Rx_Data[16] | DT7_Rx_Data[17] << 8) - 1024;

		if (RC_CtrlData.wheel < -660 || RC_CtrlData.wheel > 660)
		{
			RC_CtrlData.wheel = 0;
		}

		/*�������ݴ���*/
		KeyMouseFlag_Update();

		if (RC_CtrlData.rc.s1 == RC_SW_UP && RC_CtrlData.rc.s2 != 0)
		{
			ControlMes.x_velocity = -0.5 * RC_CtrlData.rc.ch3; // ��������
			ControlMes.y_velocity = -0.5 * RC_CtrlData.rc.ch2; // ��������
			chassis_control.Speed_ToCloud.vx = -0.5 * RC_CtrlData.rc.ch3;
			chassis_control.Speed_ToCloud.vy = -0.5 * RC_CtrlData.rc.ch2;
			chassis_control.Speed_ToCloud.w = 0.5 * RC_CtrlData.wheel * 3;
			if (RC_CtrlData.rc.s2 == RC_SW_DOWN)
				climb_flag = 4; // ���ַ���

			if (RC_CtrlData.rc.s2 == RC_SW_UP)
				climb_flag = 3; // ǰ�ַ���
		}

		/**************************** control code ****************************/
		// 3508�����ķ�ֵ����ٶȻ�ȡ
		/*ͨ��1*/
		/*������ң�أ������飻�ϼ���*/
		if (RC_CtrlData.rc.s1 == RC_SW_MID && RC_CtrlData.rc.s2 != 0)
		{
			/******************************ң������ֵ����******************************/
			// �����˶�����
			ControlMes.x_velocity = -0.5 * RC_CtrlData.rc.ch3; // ��������
			ControlMes.y_velocity = -0.5 * RC_CtrlData.rc.ch2; // ��������
			chassis_control.Speed_ToCloud.vx = -0.5 * RC_CtrlData.rc.ch3;
			chassis_control.Speed_ToCloud.vy = -0.5 * RC_CtrlData.rc.ch2;
			chassis_control.Speed_ToCloud.w = -2 * RC_CtrlData.wheel ;

			static int countFric = 0;
			// ����״̬���ã�UP ���� �� MID ��ֹ���䣻DOWN ��¼��

			// �Զ��壺�ҿ���
			if (RC_CtrlData.rc.s2 == RC_SW_DOWN)
			{
				ControlMes.modelFlag = model_Record;
				ControlMes.fric_Flag = 0;
				countFric = 0;
				ControlMes.shoot_state = RC_SW_MID;
				climb_flag = 2;
			}
			else if (RC_CtrlData.rc.s2 == RC_SW_MID)
			{
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 0;
				ControlMes.shoot_state = RC_CtrlData.rc.s2;
				countFric = 0;
			}
			else if (RC_CtrlData.rc.s2 == RC_SW_UP)
			{
				ControlMes.modelFlag = model_Normal;
				ControlMes.fric_Flag = 1;
				if (countFric < 50)
				{
					countFric++;
				}
				else
				{
					ControlMes.shoot_state = RC_CtrlData.rc.s2;
				}
				climb_flag = 1;
			}

			// ��̨�˶�����
			ControlMes.AutoAimFlag = 0;
			ControlMes.pitch_velocity = -RC_CtrlData.rc.ch1;		// ��������
			ControlMes.yaw_velocity = -RC_CtrlData.rc.ch0;			// ��������
			ControlMes.z_rotation_velocity = RC_CtrlData.wheel * 3; // ��������
			// ControlMes.yaw_position = Auto_Aim_Yaw;
		}

		else if (RC_CtrlData.rc.s1 == RC_SW_DOWN && RC_CtrlData.rc.s2 != 0)
		{
			/******************************ң������ֵ����******************************/
			// �����˶�����
			ControlMes.x_velocity = RC_CtrlData.rc.ch3; // ��������
			ControlMes.y_velocity = RC_CtrlData.rc.ch2; // ��������
			
			climb_flag=0;

			// ����״̬���ã�UP ����ģʽ �� MID ��ֹ���䣻DOWN ����ģʽ��
			ControlMes.shoot_state = RC_CtrlData.rc.s2;

			// ������̨�˶�����
			ControlMes.pitch_velocity = RC_CtrlData.rc.ch1 * 0.5; // ��������
			ControlMes.yaw_velocity = RC_CtrlData.rc.ch0;		  // ��������
			ControlMes.AutoAimFlag = 1;
		}
		else if (RC_CtrlData.rc.s1 == RC_SW_UP && RC_CtrlData.rc.s2 == RC_SW_MID)
		{
			// RemoteControl_PC_Update();
		}
		else
		{
			ControlMes.AutoAimFlag = 0;
			ControlMes.x_velocity = 0;			// ��������
			ControlMes.y_velocity = 0;			// ��������
			ControlMes.z_rotation_velocity = 0; // ��������
			ControlMes.yaw_velocity = 0;
			ControlMes.pitch_velocity = 0;
			ControlMes.shoot_state = RC_SW_MID;
		}
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, DT7_Rx_Data, sizeof(DT7_Rx_Data));
}

