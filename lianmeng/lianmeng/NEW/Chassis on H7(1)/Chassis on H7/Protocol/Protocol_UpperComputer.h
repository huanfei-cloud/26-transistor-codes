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
#include "pid.h"
#include "string.h"

void UpperCom_Receive_From_Up(uint8_t Rec[]);
void UpperCom_Send_To_Up(uint8_t COM);

#define UpperCom_MAX_BUF  25

extern struct Struct_PID_Manage_Object Auto_Aim_PID;
extern float Auto_Aim_Yaw;
extern float Auto_Aim_Pitch;

#endif
