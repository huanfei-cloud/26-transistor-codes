/**
 * @file DT7.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-4
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __DT7_H
#define __DT7_H
 
#include "main.h"
#include "usart.h"
#include "Mecanum_Chassis.h"
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "Protocol_UpperComputer.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* --------------------------- Climb Status -------------------------------- */
#define CLIMB_EXTERN ((uint16_t)1)         //伸出
#define CLIMB_WITHDRAW ((uint16_t)0)       //收回
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)

#define RC_FRAME_LENGTH 18u

#define KEYMOUSE_AMOUNT 18 //键盘鼠标总和：18个键。
#define IT_KEYMOUSE_AMOUNT 18 //图传键盘鼠标总和：18个键。

#define TIME_KeyMouse_Press 5 //超过该时间视为 按下。
//在两者之间视为 单击
#define TIME_KeyMouse_LongPress 60 //超过该时间视为 长按

#define DT7_FunGroundInit           \
    {                                \
            &DT7_Init, 				 \
            &DT7_Handle,             \
			&DR16_DataCheck,		 \
			&Online_Check_DR16,			 \
    }

#define DR16_ExportDataGroundInit \
    {                             \
        {0, 0},                   \
            {0, 0, 0, {0}},       \
            {0, 0, 0, 0, 0, 0},   \
            0,                    \
            0,                    \
    }
		
#define Image_Transmission_ExportDataGroundInit \
    {                             \
        {0, 0},                   \
            {0, 0, 0, {0}},       \
            {0, 0, 0, 0, 0, 0},   \
            0,                    \
            0,                    \
    }



/* ----------------------- Data Struct ------------------------------------- */
	
typedef struct
{
	void (*DT7_Init)(void);
    void (*DT7_Handle)(void);
	int (*DR16_DataCheck)(void);
	void (*Online_Check_DR16)(void);
} DT7_Fun_t;
	
/**
  * @brief  remote control information
  */
typedef  struct
{
	struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	  /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } key;
	
	int16_t wheel;
	
	uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
}RC_Ctl_t;

typedef enum
{
    //与DR16_Export_data.KeyMouse 的flag位一一对应
    KEY_W = 0,
    KEY_S = 1,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    MOUSE_Left,
    MOUSE_Right
} KeyList_e;

typedef enum
{
    KeyAction_CLICK,
    KeyAction_PRESS,
    KeyAction_LONG_PRESS
} KeyAction_e; //鼠标键盘（键）事件类型。

//DT7控制链路
typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                //键鼠按下标志
        uint32_t Click_Press_Flag;          //键鼠单击标志
        uint32_t Long_Press_Flag;           //键鼠长按标志
        uint8_t PressTime[KEYMOUSE_AMOUNT]; //键鼠按下持续时间
    } KeyMouse;                             //鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} DR16_Export_Data_t;         //供其他文件使用的输出数据。

//图传控制链路
typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                //键鼠按下标志
        uint32_t Click_Press_Flag;          //键鼠单击标志
        uint32_t Long_Press_Flag;           //键鼠长按标志
        uint8_t PressTime[KEYMOUSE_AMOUNT]; //键鼠按下持续时间
    } KeyMouse;                             //鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} Image_Transmission_Export_Data_t;         //供其他文件使用的输出数据。


extern RC_Ctl_t RC_CtrlData;
extern uint8_t DT7_RX_Finish;
extern uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];
extern DT7_Fun_t DT7_Fun;

#endif
