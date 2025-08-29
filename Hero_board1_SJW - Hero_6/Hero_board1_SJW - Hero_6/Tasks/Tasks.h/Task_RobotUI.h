/**
 * @file Task_RobotUI.h
 * @author Cyx(1696143358@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-19
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "UI.h"
#include "typedef.h"
#include "BSP_BoardCommunication.h"
#include "M6020_Motor.h"
#include "Saber_C3.h"

extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String1;
extern UI_String_t UI_String2;
extern UI_String_t UI_String3;
extern UI_String_t UI_String4;
extern UI_String_t UI_String5;
extern UI_Delete_t UI_Delete;

void Robot_UI(void const *argument);
