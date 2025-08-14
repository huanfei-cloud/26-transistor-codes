#include "PowerBank.h"

struct TxData PowerTxData;
RxData PowerRxData;

void PowerBankSend(void)
{
	PowerTxData.enableDCDC=1;
//	if(PowerRxData.errorCode != 0)
//	{
//		
//		PowerTxData.systemRestart = 1;
//	}
//	else{
//	PowerTxData.systemRestart=0;
//	}
	PowerTxData.systemRestart = 0;
	PowerTxData.feedbackRefereePowerLimit = ext_game_robot_state.data.chassis_power_limit;
	PowerTxData.feedbackRefereeEnergyBuffer = ext_power_heat_data.data.chassis_power_buffer;
//	PowerTxData->feedbackRefereePowerLimit =37;
//	PowerTxData->feedbackRefereeEnergyBuffer = 0;
	fdcanx_send_data(&hfdcan1, 0x61, (uint8_t*)&PowerTxData, 8);
}	

void PowerBankReceive(uint8_t *RxMessage)
{
  PowerRxData.errorCode = RxMessage[0];
	uint32_t temp = (RxMessage[4] << 24) | (RxMessage[3] << 16) | (RxMessage[2] << 8) | (RxMessage[1]);
	PowerRxData.chassisPower = * ((float *)&temp);
	PowerRxData.chassisPowerLimit = (uint16_t) (RxMessage[5]) | (RxMessage[6]);
	PowerRxData.capEnergy = RxMessage[7];
}

