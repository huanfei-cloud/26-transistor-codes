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

#define TIME_KeyMouse_Press 5 //������ʱ����Ϊ ���¡�
//������֮����Ϊ ����
#define TIME_KeyMouse_LongPress 60 //������ʱ����Ϊ ����