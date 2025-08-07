/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @version 0.1
 * @date 2023-10-02
 *
 */

#ifndef PROTOCOL_UPPERCOMPUTER_H
#define PROTOCOL_UPPERCOMPUTER_H

#include "CRC.h"
#include "usart.h"
#include "PID.h"

void UpperCom_Receive_From_Up(uint8_t Rec[]);
void UpperCom_Send_To_Up(uint8_t COM);

#define UpperCom_MAX_BUF  25
#define Test_Pitch_SEN  22.75556       
#define Test_Yaw_SEN  22.75556           //8192/360 = 22.75556

extern positionpid_t Auto_Aim_PID;
extern float Auto_Aim_Yaw;
extern float Auto_Aim_Pitch;

#endif
