#ifndef BSP_TEST_H
#define BSP_TEST_H

#include "struct_typedef.h"
#include "main.h"
#include "BSP_Usart.h"
//#include "Omni_Chassis.h"
#include "M6020_Motor.h"

#define FIRE_MAX_BUF_LEN 125 //宏定义：最长数据长度

//上位机-->下位机指令宏定义
#define SET_PID 0x10
#define SET_Target 0x11
/* 数据头结构体 */
typedef __packed struct
{
  uint32_t head;    // 包头
  uint8_t ch;       // 通道
  uint32_t len;     // 包长度
  uint8_t cmd;      // 命令
//  uint8_t sum;      // 校验和
  
}Packet_Head_t;

#define FIRE_MAX_BUF_LEN 125 //宏定义：最长数据长度

#define FRAME_HEADER     0x59485A53    // 帧头

//上位机-->下位机指令宏定义
#define SET_PID 0x10 //上位机设置下位机PID
#define SET_Target 0x11 //上位机设置下位机目标值

//下位机-->上位机指令宏定义
#define RETURN_Target 0x01 //下位机设置上位机通道的目标值
#define RETURN_RealValue 0x02 //下位机设置上位机通道的实际值
#define RETURN_Cycletime 0x06 //下位机设置上位机通道的周期

	extern uint8_t Fire_Rx_Data[FIRE_MAX_BUF_LEN];//野火串口调试数据缓冲区
	extern uint8_t Fire_RxBuffer[FIRE_MAX_BUF_LEN];//野火串口调试数据存储区
	extern uint16_t Fire_Bx_Len;
	extern uint8_t Fire_Rx_Finish;//野火串口一包数据接收完成标志 
		
	
	void Fire_Debug_Init(void);//野火串口调试初始化。
	void Fire_Handle(void);//野火串口数据处理。
	uint8_t Fire_Head_Judge(uint8_t *data);//野火串口帧头判断
	uint8_t Fire_Cmd(uint8_t *data);//野火指令类型判断
	void Fire_Set_Computer_Value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);//设置上位机的值
	uint8_t Check_Sum(uint8_t init, uint8_t *ptr, uint8_t len );//计算校验和
	
#endif
