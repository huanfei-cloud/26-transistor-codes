/**
 * @file TDF.h
 * @author SJW
 * @brief
 * @version .01
 * @date 2025-9-23
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _TDF_H_
#define _TDF_H_

#include "main.h"
#include "usart.h"
#include "string.h"
#include <stdint.h>
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "Protocol_UpperComputer.h"
#include "Dial.h"
#include "Shoot.h"
#include "Cloud_Control.h"
#include "SpeedRamp.h"
#include "DT7.h"

#define SBUS_RX_LEN 25 // 25
#define StartByte 0x0f
#define EndByte 0x00
// SBUS原始数据范围（典型值）
#define SBUS_MIN 353
#define SBUS_MAX 1695
// 中间阈值（用于档位判断）
#define MID_VALUE 1024
// 档位判断容差（避免数据抖动导致跳变）
#define TOLERANCE 50
#define RC_UP ((uint16_t)3)
#define RC_MID ((uint16_t)2)
#define RC_DOWN ((uint16_t)1)
#define KEYMOUSE_AMOUNT 18    // 键盘鼠标总和：18个键。
#define IT_KEYMOUSE_AMOUNT 18 // 图传键盘鼠标总和：18个键。
#define TIME_KeyMouse_Press 3 // 超过该时间视为 按下。
// 在两者之间视为 单击
#define TIME_KeyMouse_LongPress 60 // 超过该时间视为 长按

#define TDF_ExportDataGroundInit \
    {                            \
        {0, 0},                  \
        {0, 0, 0, {0}},          \
        {0, 0, 0, 0, 0, 0},      \
        0,                       \
        0,                       \
    }


/* -------------------- Data Struct ------------------------------------- */
/**
 * @brief  remote control information
 */
typedef struct
{
    uint8_t Start;
    uint16_t Ch1;
    uint16_t Ch2;
    uint16_t Ch3;
    uint16_t Ch4;
    uint16_t Ch5;
    uint16_t Ch6;
    uint16_t Ch7;
    uint16_t Ch8;
    uint16_t Ch9;
    uint16_t Ch10;
    uint16_t Ch11;
    uint16_t Ch12;
    uint16_t Ch13;
    uint16_t Ch14;
    uint16_t Ch15;
    uint16_t Ch16;
    uint8_t Flag;
    uint8_t End;
} SBUS_Buffer;

/* ----------------------- Data Struct ------------------------------------- */
/**
 * @brief  remote control information
 */
typedef struct
{
    struct
    {
        int16_t Ch1;  // -1024~1024
        int16_t Ch2;  // -1024~1024
        int16_t Ch3;  // -1024~1024
        int16_t Ch4;  // -1024~1024
        uint8_t Ch5;  // 1/2/3档
        uint8_t Ch6;  // 1/2/3档
        uint8_t Ch7;  // 1/2/3档
        uint8_t Ch8;  // 1/2/3档
        uint8_t Ch9;  // 1/2/3档
        uint8_t Ch10; // 1/2/3档
        uint8_t Ch11; // 1/2档
        uint8_t Ch12; // 1/2档
        int16_t Ch13; // -1024~1024
        int16_t Ch14; // -1024~1024
        int16_t Ch15; // -1024~1024
        int16_t Ch16; // -1024~1024
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

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

} RC_Ctl_TDF_t;

typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {
        uint32_t Press_Flag;                // 键鼠按下标志
        uint32_t Click_Press_Flag;          // 键鼠单击标志
        uint32_t Long_Press_Flag;           // 键鼠长按标志
        uint8_t PressTime[KEYMOUSE_AMOUNT]; // 键鼠按下持续时间
    } KeyMouse;                             // 鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; // Vx
        float Omega_Value;        // 自旋值。
        float Left_Right_Value;   // Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel;     // 拨轮
    } Robot_TargetValue;      // 遥控计算比例后的运动速度
    uint16_t infoUpdateFrame; // 帧率
    uint8_t OffLineFlag;      // 设备离线标志
} TDF_Export_Data_t;          // 供其他文件使用的输出数据。

extern uint8_t SBUS_RX_Finish;
extern uint8_t SBUS_RXBuffer[SBUS_RX_LEN];
extern uint8_t SBUS_Rx_Data[25];

void TDF_Init(void);
void TDF_Handle(void);
#endif
