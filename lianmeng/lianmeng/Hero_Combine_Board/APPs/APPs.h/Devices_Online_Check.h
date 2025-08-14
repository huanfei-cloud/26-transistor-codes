/**
 * @file Devices_Online_Check.h
 * @author Lxr
 * @brief
 * @version 1.0
 * @date 2024-01-23
 *
 * @copyright Copyright (c) 2024
 *
 */
 #ifndef __DEVICES_ONLINE_CHECK_H
 #define __DEVICES_ONLINE_CHECK_H
 
 
#include "tim.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define DevicesMonitor_FUNGroundInit \
    {                                \
        &DevicesMonitor_update,      \
            &DevicesInit,            \
    }

typedef enum
{
	#if(USING_BOARD == BOARD1)
    CHECK_M3508 = 0,
    CHECK_DR16 = 1,
   // CHECK_Supercapacitor,
    CHECK_M6020,
	#else
	CHECK_M3508 = 0,
	CHECK_Saber,
	CHECK_M6020,
	#endif
   // CHECK_M2006,
   // CHECK_M3508F,
   // CHECK_Vision,
   // CHECK_CAN,
   // CHECK_Judge,
    //数组下标
    CHECK_LIST_Device,
} CHECK_Devices_t;

typedef struct
{
    void (*GetDevices)(void);
} Check_Fun_t;

typedef struct
{
    void (*DevicesMonitor_update)(void);
    void (*DevicesMonitor_Init)(void);
    //void (*Buzzer_On)(bool on, int volume);
    void (*DevicesMonitor_Alert)(void);
    void (*DevicesInit)(void);
} DevicesMonitor_FUN_t;

extern DevicesMonitor_FUN_t DevicesMonitor_FUN;

#endif
