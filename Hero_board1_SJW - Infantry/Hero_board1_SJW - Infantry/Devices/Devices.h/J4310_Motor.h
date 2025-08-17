/**
 * @file D4310_Motor.h
 * @author ZS
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c)
 *
 */
#ifndef __J4310_MOTOR_H
#define __J4310_MOTOR_H

#include "can.h"
#include "main.h"
#include "typedef.h"
#include "Task_CanReceive.h"
#include "PID.h"
#include "BSP_Can.h"
#include "stm32f4xx_hal_can.h"

#define J4310_READID_PITCH 0x01
#define J4310_SENDID_Pitch 0x001
#define J4310_MaxV 200
#define J4310_MaxT 7
#define J4310_ReductionRatio 10

#define J4310_FIRSTANGLE 3800

#define J4310_mAngleRatio 22.7527f /
#define Pi 3.14159265

#define J4310_getRoundAngle(rAngle) rAngle / J4310_mAngleRatio

#define J4310_FunGroundInit  \
  {                          \
      &J4310_setParameter,   \
      &J4310_Enable,         \
      &J4310_Save_Pos_Zero,  \
      &J4310_getInfo,        \
      &J4310_setTargetAngle, \
      &J4310_Reset,          \
      &Check_J4310,          \
  }

typedef struct
{
  int16_t state;
  float realAngle;
  float realSpeed;
  uint8_t temperatureMOS;
  uint8_t temperatureRotor;
  float torqueInit;
  float torque;
  float angleInit;
  float speedInit;

  uint16_t lastAngle;

  int32_t targetSpeed;
  int32_t targetAngle;

  float outPosition;
  float outSpeed;
  float outTorque;

  int16_t turnCount;
  float totalAngle;

  int8_t outKp;
  int8_t outKd;

  uint8_t InfoUpdateFlag;
  uint16_t InfoUpdateFrame;
  uint8_t OffLineFlag;
} J4310s_t;

extern J4310s_t J4310s_Pitch;

typedef enum
{
  J4310_PITCH = 0,
} J4310Name_e;

typedef struct
{
  void (*J4310_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
  void (*J4310_Enable)(void);
  void (*J4310_Save_Pos_Zero)(void);
  void (*J4310_getInfo)(Can_Export_Data_t RxMessage);
  void (*J4310_setTargetAngle)(J4310s_t *J4310, int32_t angle);
  void (*J4310_Reset)(J4310s_t *J4310);
  void (*Check_J4310)(void);
} J4310_Fun_t;

extern J4310s_t J4310s_Pitch;
extern J4310_Fun_t J4310_Fun;

#endif /* __J4310_MOTOR_H */
