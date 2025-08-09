/**
 * @file PowerControl.c
 * @author Why
 * @brief ?????????????
 * @version 0.1
 * @date 2023-08-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H
#include "Protocol_Judgement.h"
#include "BSP_can.h"
#include "Extern_Handles.h"
#define PowerControl_FunGroundInit      \
    {                                   \
		&PowerControl_MsgSend,			\
		&PowerControl_MsgRec,			\
    }
typedef struct
{
    uint8_t errorCode;
    float chassisPower;
    uint16_t chassisPowerLimit;
    uint8_t capEnergy;
} RxData;
		
struct TxData
{
    uint8_t enableDCDC : 1;
    uint8_t systemRestart : 1;
    uint8_t resv0 : 6;
    uint16_t feedbackRefereePowerLimit;
    uint16_t feedbackRefereeEnergyBuffer;
    uint8_t resv1[3];
} __attribute__((packed)); 

extern struct TxData PowerTxData;
extern RxData PowerRxData;

typedef struct
{
	void (*PowerControl_MsgSend)(void);
	void (*PowerControl_MsgRec)(Can_Export_Data_t RxMessage);
} PowerControl_Fun_t;

extern PowerControl_Fun_t PowerControl_Fun;

#endif /*__POWERCONTROL_H*/