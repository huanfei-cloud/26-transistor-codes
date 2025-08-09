#ifndef __POWER_H
#define __POWER_H
#include "struct_typedef.h"
#include "can.h"
#include "string.h"
struct TxData
{
    // struct powerTx
    // {
    uint8_t enableDCDC : 1;
    uint8_t systemRestart : 1;
    uint8_t resv0 : 6;
    uint16_t feedbackRefereePowerLimit;
    uint16_t feedbackRefereeEnergyBuffer;
    uint8_t resv1[3];
    // }powerTx;
    // uint8_t temp[8];
} __attribute__((packed));

union RxData
{
    struct
    {
        uint8_t errorCode;
        float chassisPower;
        uint16_t chassisPowerLimit;
        uint8_t capEnergy;
    }__attribute__((packed)) powerRx;
    uint8_t temp[8];
    // uint8_t errorCode;
    // float chassisPower;
    // uint16_t chassisPowerLimit;
    // uint8_t capEnergy;
};

typedef struct
{
    CAN_HandleTypeDef *p_hfdcan;
    CAN_TxHeaderTypeDef Header;
    TxData Data;
} FDCAN_Power_TxFrame;

// typedef struct
// {
//     FDCAN_HandleTypeDef *p_hfdcan;
//     FDCAN_RxHeaderTypeDef Header;
//     RxData Data;
// } FDCAN_Power_RxFrame;

class Power
{
public:
    Power(FDCAN_HandleTypeDef *hfdcan, uint32_t txID, uint32_t rxID) : p_hfdcan_(hfdcan),
                                                                       txID_(txID),
                                                                       rxID_(rxID)
    {
        TxFrame_.p_hfdcan = hfdcan;
        TxFrame_.Header.Identifier = txID;
        TxFrame_.Header.IdType = FDCAN_STANDARD_ID;     // ID类型
        TxFrame_.Header.TxFrameType = FDCAN_DATA_FRAME; // 发送的为数据
        TxFrame_.Header.DataLength = FDCAN_DLC_BYTES_8; // 数据长度为8字节
        TxFrame_.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxFrame_.Header.BitRateSwitch = FDCAN_BRS_OFF;
        TxFrame_.Header.FDFormat = FDCAN_CLASSIC_CAN;
        TxFrame_.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        TxFrame_.Header.MessageMarker = 0;
    }
    void output(uint16_t RefereePowerLimit, uint16_t RefereeEnergyBuffer);
    void update(FDCAN_RxFrame *RxFrame);
    uint32_t getRxID();
    uint32_t getTxID();

private:
    FDCAN_HandleTypeDef *p_hfdcan_;
    uint32_t txID_;
    uint32_t rxID_;
    uint16_t RefereePowerLimit_;
    uint16_t RefereeEnergyBuffer_;

    uint8_t errorCode_;
    float chassisPower_;
    uint16_t chassisPowerLimit_;
    uint8_t capEnergy_;

    FDCAN_Power_TxFrame TxFrame_;
};

#endif