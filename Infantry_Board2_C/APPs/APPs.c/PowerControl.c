#include "PowerControl.h"
void PowerControl_MsgSend(void);
void PowerControl_MsgRec(Can_Export_Data_t RxMessage);

struct TxData PowerTxData;
RxData PowerRxData;

 
PowerControl_Fun_t PowerControl_Fun = PowerControl_FunGroundInit;
#undef PowerControl_FunGroundInit

void PowerControl_MsgSend(void)
{
    PowerTxData.enableDCDC = 1;
    PowerTxData.systemRestart = 0;
    PowerTxData.feedbackRefereePowerLimit = g_referee.robot_status_.chassis_power_limit;
    PowerTxData.feedbackRefereeEnergyBuffer = g_referee.power_heat_.buffer_energy;
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, 0x61, (uint8_t*)&PowerTxData);
}

void PowerControl_MsgRec(Can_Export_Data_t RxMessage)
{
    PowerRxData.errorCode = RxMessage.CANx_Export_RxMessage[0];
    uint32_t temp = (RxMessage.CANx_Export_RxMessage[4] << 24) | (RxMessage.CANx_Export_RxMessage[3] << 16) | (RxMessage.CANx_Export_RxMessage[2] << 8) | (RxMessage.CANx_Export_RxMessage[1]);
    PowerRxData.chassisPower = *((float *)&temp);
    PowerRxData.chassisPowerLimit = (uint16_t) (RxMessage.CANx_Export_RxMessage[5]) | (RxMessage.CANx_Export_RxMessage[6]);
    PowerRxData.capEnergy = RxMessage.CANx_Export_RxMessage[7];
}

///**
//  * @brief  ????????????
//  * @param
//  * @retval void
//  */
//void PowerControl_MsgSend()
//{
//    static uint8_t time;
//    static uint8_t data[8];
//    /* ???????,?????????? */
//    if (time++ <= 20) return;
//    time = 0;

//    /* ??0x2E????????? */
//    memset(data, 0, 8);
//	data[0] = PowerControl_Data.PoweBuffer >> 8;
//    data[1] = PowerControl_Data.PoweBuffer;
//    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, PowerBuffer_ID, data);

//	PowerControl_Data.Power_Limit = 40;
//    /* ??0x2F??????????? */
//    data[0] = PowerControl_Data.Power_Limit >> 8;
//    data[1] = PowerControl_Data.Power_Limit;
//    data[2] = PowerControl_Data.DischargeLimit >> 8;
//    data[3] = PowerControl_Data.DischargeLimit;
//    data[4] = PowerControl_Data.ChargeLimit >> 8;
//    data[5] = PowerControl_Data.ChargeLimit;
//    data