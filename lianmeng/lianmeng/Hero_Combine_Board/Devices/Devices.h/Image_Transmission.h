/**
 * @file Image_Transmission.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-4
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
#include "main.h"
#include "usart.h"
#include "Mecanum_Chassis.h"
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "Protocol_UpperComputer.h"

#define TIME_KeyMouse_Press 5 //超过该时间视为 按下。
//在两者之间视为 单击
#define TIME_KeyMouse_LongPress 60 //超过该时间视为 长按