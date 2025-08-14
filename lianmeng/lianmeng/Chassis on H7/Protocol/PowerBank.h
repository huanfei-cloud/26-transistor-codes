#ifndef POWER_BANK_H
#define POWER_BANK_H

#include "main.h"
#include "PowerControl.h"
#include "Protocol_Judgement.h"
#include "bsp_fdcan.h"

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

void PowerBankSend(void);
void PowerBankReceive(uint8_t *RxMessage);

#endif

