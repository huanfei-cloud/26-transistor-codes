/**
 * @file D4340_Motor.h
 * @author SJW
 * @brief
 * @version 0.1
 * @date 2025-8-19
 *
 * @copyright Copyright (c)
 *
 */
#ifndef __J4340_MOTOR_H
#define __J4340_MOTOR_H

#include "can.h"
#include "main.h"
#include "typedef.h"
#include "Task_CanReceive.h"
#include "PID.h"
#include "BSP_Can.h"
#include "stm32f4xx_hal_can.h"

#define J4340_READID_PITCH 0x01
#define J4340_SENDID_Pitch 0x001
#define J4340_MaxV 200
#define J4340_MaxT 7
#define J4340_ReductionRatio 10

#define J4340_FIRSTANGLE 3800

#define J4340_mAngleRatio 22.7527f /
#define Pi 3.14159265f

#define J4340_getRoundAngle(rAngle) rAngle / J4340_mAngleRatio

#define J4340_FunGroundInit    \
    {                          \
        &J4340_setParameter,   \
        &J4340_Enable,         \
        &J4340_Save_Pos_Zero,  \
        &J4340_getInfo,        \
        &J4340_setTargetAngle, \
        &J4340_Reset,          \
        &Check_J4340,          \
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
} J4340s_t;

extern J4340s_t J4340s_Pitch;

typedef enum
{
    J4340_PITCH = 0,
} J4340Name_e;

typedef struct
{
    void (*J4340_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5, uint8_t *data);
    void (*J4340_Enable)(void);
    void (*J4340_Save_Pos_Zero)(void);
    void (*J4340_getInfo)(Can_Export_Data_t RxMessage);
    void (*J4340_setTargetAngle)(J4340s_t *J4340, int32_t angle);
    void (*J4340_Reset)(J4340s_t *J4340);
    void (*Check_J4340)(void);
} J4340_Fun_t;

extern J4340s_t J4340s_Pitch;
extern J4340_Fun_t J4340_Fun;

#endif /* __J4340_MOTOR_H */
