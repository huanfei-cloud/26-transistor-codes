#include "DT7.h"

/* ----------------------- Internal Data ----------------------------------- */
RC_Ctl_t RC_CtrlData;
static uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];
uint8_t DT7_RX_Finish;
int follow_flag;
/**
  * @brief  DT7��ʼ�����������տ����ж�
  * @param  void
  * @retval void
  */
void DT7_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DT7_Rx_Data,sizeof(DT7_Rx_Data));
}

/**
  * @brief  DT7���ݽ���
  * @param 	void
  * @retval void
  */
void DT7_Handle(void)
{
//	if(DT7_RX_Finish == 1)
//	{
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
		if (RC_CtrlData.rc.ch0 <= 5 && RC_CtrlData.rc.ch0 >= -5)
		{
			RC_CtrlData.rc.ch0 = 0;
		}
		if (RC_CtrlData.rc.ch1 <= 5 && RC_CtrlData.rc.ch1 >= -5)
		{
			RC_CtrlData.rc.ch1 = 0;
		}
		if (RC_CtrlData.rc.ch2 <= 5 && RC_CtrlData.rc.ch2 >= -5)
		{
			RC_CtrlData.rc.ch2 = 0;
		}
		if (RC_CtrlData.rc.ch3 <= 5 && RC_CtrlData.rc.ch3 >= -5)
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

		RC_CtrlData.key.v = DT7_Rx_Data[14] | DT7_Rx_Data[15] << 8; // key borad code
		RC_CtrlData.wheel = (DT7_Rx_Data[16] | DT7_Rx_Data[17] << 8) - 1024;
		
		if(RC_CtrlData.wheel< -660 || RC_CtrlData.wheel > 660)
		{
			RC_CtrlData.wheel = 0;
		}
	
		//control code 	
		//��ȫ��������ٶȻ�ȡ
		if(RC_CtrlData.rc.s1 == RC_SW_MID)
		{
			Steer_Omni_Data.Speed_ToCloud.vx = RC_CtrlData.rc.ch3; //��������
			Steer_Omni_Data.Speed_ToCloud.vy = RC_CtrlData.rc.ch2; //��������
			Steer_Omni_Data.Speed_ToCloud.wz = -1*RC_CtrlData.wheel/100; //����
			Cloud.Target_Yaw+=RC_CtrlData.wheel * 0.0005 * 25;
			if(RC_CtrlData.rc.s2 == RC_SW_DOWN)
			{
				follow_flag = 1; 
			}
			else
			{
				follow_flag = 0; 
			}
		}
		else
		{
			Steer_Omni_Data.Speed_ToCloud.vx = 0; //��������
			Steer_Omni_Data.Speed_ToCloud.vy = 0; //
			Steer_Omni_Data.Speed_ToCloud.wz = 0; //
		}
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DT7_Rx_Data,sizeof(DT7_Rx_Data));
	
//	}
}
