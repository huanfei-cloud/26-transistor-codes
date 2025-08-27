/**
 * @file Task_RobotControl.c
 * @author Rm_Team
 * @brief
 * @version 0.1
 * @date 2023-08-30
 *
 * @copyright Copyright (c) 2021
 *
 */ 
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "PowerControl.h"
#include "Steer_Omni_Chassis.h"
#include "Cloud_Control.h"
#include "Saber_C3.h"
#include "DT7.h"

void Robot_Control(void const *argument);
