/**
 * @file UI.c
 * @author Cyx (1686143358@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */
 
#ifndef __UI_H__
#define __UI_H__

#include "Protocol_Judgement.h"
#include "struct_typedef.h"
#include "usart.h"
#include "freeRTOS.h"
#pragma pack(1)                           //按1字节对齐

#define NULL 0
#define M__FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方客户端ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方客户端ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***********************图形操作********************/
#define UI_Graph_Add 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点数
#define UI_Graph_Int 6          //整型
#define UI_Graph_Char 7         //字符串
/***************************图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8


typedef struct
{
   uint8_t SOF;                    //起始字节,固定0xA5
   uint16_t Data_Length;           //帧数据长度
   uint8_t Seq;                    //包序号
   uint8_t CRC8;                   //CRC8校验值
   uint16_t CMD_ID;                //命令ID
} UI_Packhead;             //帧头

typedef struct
{
   uint16_t Data_ID;               //内容ID
   uint16_t Sender_ID;             //发送者ID
   uint16_t Receiver_ID;           //接收ID
} UI_Data_Operate;         //操作定义帧

typedef struct
{
   uint8_t Delete_Operate;         //删除操作
   uint8_t Layer;                  //删除图层
} UI_Data_Delete;          //删除帧

typedef struct
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   int32_t graph_Float;              //浮点数据
} Float_Data;

typedef struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11;
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11;              //图形数据
} Graph_Data;


typedef struct
{
   Graph_Data Graph_Control;
   char show_Data[30];
} String_Data;                  //字符串数据

typedef struct
{
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	Graph_Data imageData[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef struct
{
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	Graph_Data imageData[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef struct
{
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	Graph_Data imageData[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef struct
{
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	Graph_Data imageData[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef struct
{ 
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	String_Data String;
	uint16_t CRC16;
}
UI_String_t;

typedef struct
{
	UI_Packhead framehead;
	UI_Data_Operate datahead;
	UI_Data_Delete Delete;
	uint16_t CRC16;
} UI_Delete_t;

//extern TaskHandle_t UI_Handle;

void ID_Judge(void);
void UI_init(void);
void UI_PushUp_Graphs(uint8_t cnt, void *Graphs);
void UI_PushUp_String(UI_String_t *String);
void UI_PushUp_Delete(UI_Delete_t *Delete);
void Line_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Circle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius);
void Rectangle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
void Float_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float FloatData);
void Char_Draw(String_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data);
void Arc_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length);


typedef struct
{
	void (*ID_Judge)(void);
	void (*UI_init)(void);
	void (*UI_PushUp_Graphs)(uint8_t cnt, void *Graphs);
	void (*UI_PushUp_String)(UI_String_t *String);
	void (*UI_PushUp_Delete)(UI_Delete_t *Delete);
	void (*Line_Draw)(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
	unsigned char (*Get_CRC8_Check_Sum_UI)(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
	uint16_t (*Get_CRC16_Check_Sum_UI)(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
	void (*Circle_Draw)(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius);
	void (*Rectangle_Draw)(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
	void (*Float_Draw)(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float FloatData);
	void (*Char_Draw)(String_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data);
	void (*Arc_Draw)(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length);
		
} UI_FUN_t;


#define UI_FUNGroundInit               \
    {                                     \
			&ID_Judge,			  \
      &UI_init,                      \
			&UI_PushUp_Graphs,			  \
      &UI_PushUp_String,                      \
			&UI_PushUp_Delete,			  \
      &Line_Draw,                      \
			&Get_CRC8_Check_Sum_UI,			  \
      &Get_CRC16_Check_Sum_UI,                      \
			&Circle_Draw,			  \
      &Rectangle_Draw,                      \
			&Float_Draw,			  \
      &Char_Draw,                      \
			&Arc_Draw,			  \
    }



extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String1;
extern UI_String_t UI_String2;
extern UI_String_t UI_String3;
extern UI_Delete_t UI_Delete;

extern UI_FUN_t UI_FUN;
		
#pragma pack()                           //按1字节对齐

#endif
