#include "SBUS.h"
#include "string.h"

SBUS_Buffer SBUS;
uint8_t SBUS_RX_Finish=0;
uint8_t SBUS_RXBuffer[SBUS_RX_LEN] = {0};//接收缓冲
uint8_t SBUS_Rx_Data[25];
/**
  * @brief  SBUS初始化，开启接收空闲中断
  * @param  void
  * @retval void
  */
void SBUS_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,SBUS_Rx_Data,sizeof(SBUS_Rx_Data));
}

/**
  * @brief  检测遥控器是否在线
  * @param  void
  * @retval int
  */
int Control_Cheak()
{
	if(SBUS_RXBuffer[0] != 0x0F && SBUS_RXBuffer[24] != 0x00)
	{
		return 0;
	}
	return 1;
}

/**
  * @brief  解析遥控器数据
  * @param  void
  * @retval void
  */
void SBUS_Handle()
{
	if( SBUS_RX_Finish == 1 )
	{
		SBUS_RX_Finish=0;//准备下一次接收
		
		SBUS.Start=SBUS_RXBuffer[0];
		SBUS.Ch1=((uint16_t)SBUS_RXBuffer[1])|((uint16_t)((SBUS_RXBuffer[2]&0x07)<<8));//前进和后退通道
		SBUS.Ch2=((uint16_t)((SBUS_RXBuffer[2]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[3]&0x3f))<<6);//左平移和右平移通道
		SBUS.Ch3=((uint16_t)((SBUS_RXBuffer[3]&0xc0)>>6))|((((uint16_t)SBUS_RXBuffer[4])<<2))|(((uint16_t)(SBUS_RXBuffer[5]&0x01))<<10);//自转通道
		SBUS.Ch4=((uint16_t)((SBUS_RXBuffer[5]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[6]&0x0f))<<7);
		SBUS.Ch5=((uint16_t)((SBUS_RXBuffer[6]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[7]&0x7f))<<4);
		SBUS.Ch6=((uint16_t)((SBUS_RXBuffer[7]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[8])<<1)|(((uint16_t)(SBUS_RXBuffer[9]&0x03))<<9);
		SBUS.Ch7=((uint16_t)((SBUS_RXBuffer[9]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[10]&0x1f))<<6);
		SBUS.Ch8=((uint16_t)((SBUS_RXBuffer[10]&0xe0)>>5))|(((uint16_t)(SBUS_RXBuffer[11]))<<3);
		SBUS.Ch9=((uint16_t)SBUS_RXBuffer[12])|(((uint16_t)(SBUS_RXBuffer[13]&0x07))<<8);
		SBUS.Ch10=((uint16_t)((SBUS_RXBuffer[13]&0xf8)>>3))|(((uint16_t)(SBUS_RXBuffer[14]&0x3f))<<5);
		SBUS.Ch11=((uint16_t)((SBUS_RXBuffer[14]&0xc0)>>6))|(((uint16_t)SBUS_RXBuffer[15])<<2)|(((uint16_t)(SBUS_RXBuffer[16]&0x01))<<10);
		SBUS.Ch12=((uint16_t)((SBUS_RXBuffer[16]&0xfe)>>1))|(((uint16_t)(SBUS_RXBuffer[17]&0x0f))<<7);
		SBUS.Ch13=((uint16_t)((SBUS_RXBuffer[17]&0xf0)>>4))|(((uint16_t)(SBUS_RXBuffer[18]&0x7f))<<4);
		SBUS.Ch14=((uint16_t)((SBUS_RXBuffer[18]&0x80)>>7))|(((uint16_t)SBUS_RXBuffer[19])<<1)|(((uint16_t)(SBUS_RXBuffer[20]&0x03))<<9);
		SBUS.Ch15=((uint16_t)((SBUS_RXBuffer[20]&0xfc)>>2))|(((uint16_t)(SBUS_RXBuffer[21]&0x1f))<<6);
		SBUS.Ch16=((uint16_t)((SBUS_RXBuffer[21]&0xe0)>>5))|(((uint16_t)SBUS_RXBuffer[22])<<3);
		SBUS.Flag=SBUS_RXBuffer[23];
		SBUS.End=SBUS_RXBuffer[24];
		
//		//2006全向轮底盘速度获取
//		Omni_Data.Speed_ToCloud.vx = (SBUS.Ch1-1024)*1.5;
//		Omni_Data.Speed_ToCloud.vy = (SBUS.Ch2-2048)*0.8;
//		Omni_Data.Speed_ToCloud.vw = (SBUS.Ch3-1024)/80;
		
		//3508麦克纳姆轮底盘速度获取
		if(SBUS.Ch7 == 0x0161 && SBUS.Ch8 ==0x0161 )
		{
//			Mecanum_Data.Speed_ToCloud.vx = (SBUS.Ch1-1024)*1.5;
//			Mecanum_Data.Speed_ToCloud.vy = (SBUS.Ch2-2048)*0.8;
//			Mecanum_Data.Speed_ToCloud.vw = (SBUS.Ch3-1024)/80;
			ControlMes.x_velocity = (SBUS.Ch1-1024)*1.5;
			ControlMes.y_velocity = (SBUS.Ch2-2048)*0.8;
			ControlMes.z_rotation_velocity = (SBUS.Ch3-1024)/80;
			ControlMes.yaw_velocity = 0;
			
			//用board1 CAN2发送给board2。
			Board1_FUN.Board1_To_2();
		}
		
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,SBUS_Rx_Data,sizeof(SBUS_Rx_Data));//准备接收下一包数据
	} 
		
}
