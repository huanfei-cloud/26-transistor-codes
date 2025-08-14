/**
 * @file PowerControl.c
 * @author Why
 * @brief ����������������ƹ���֮��
 * @version 0.1
 * @date 2023-08-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H

#include "main.h"
#include "BSP_Fdcan.h"

/******�ؼ��趨��pid��3508��������ǰ�ĸ���һһ��Ӧ���ĸ�����
*******����������ط��õ�************************************/

/* ʵ�ʹ��ʾ��鹫ʽ��������Ҫʵ�ʲ�����ϣ���ʽ��Pin = Pm + Kw * w^2 +K�� * ��^2 + A */
#define PowerBuffer_ID 0x2E
#define PowerLimit_ID 0x2F
#define PowerFeedback_ID 0x30
/* ת�ص�����ȡֵ��ʵ��ת��֮���ת����������λN��m/A ����ʽ���� = TC * I��*/
#define Torque_Constant 0.0000190703f
#define It_Constant 52437.56f
/* ��е���ʳ�������ʽ��Pm = ��*w / 9.55 = ��*w*MPC */
#define MachinePower_Cnstant 0.104712042
/* ʵ�ʹ��ʾ��鹫ʽ��������Ҫʵ�ʲ�����ϣ���ʽ��Pin = Pm + Kw * w^2 +K�� * ��^2 + A */
#define Kw_d 0
#define Kt_d 0
#define A_d 0

#define Div_2Kt_d 0

#define PowerControl_FunGroundInit      \
    {                                   \
        &PowerControl_Handle,           \
		&PowerControl_MsgSend,			\
		&PowerControl_MsgRec,			\
		&PowerContol_MsgRenew,			\
    }

#define PowerControl_DataGroundInit \
    {                      \
        300,               \
        150,               \
        3,                 \
		0,                 \
        0,                 \
        0,                 \
        120,               \
        120,               \
        55,                \
    }

typedef struct
{
    /* �趨��ֵ */
    int16_t DischargeLimit;        // �������ݷŵ����ƹ���
    uint16_t ChargeLimit;          // �������ݳ�����ƹ���
    uint16_t ControlBit;           // �������ݿ���λ����һλ�ǿ��أ��ڶ�λ����ʾ����

    /* ���ص�ֵ */
    float Cap_U;                   // �����������ѹ
    float Cap_I;                   // �������������
    union                          // ����������״̬������������ݼ���˵��
    {
        int16_t Cap_Status;
        struct
        {
            uint16_t warning : 1;           //����
            uint16_t cap_v_over : 1;        //���ݹ�ѹ
            uint16_t cap_i_over : 1;        //���ݹ���
            uint16_t cap_v_low : 1;         //����Ƿѹ
            uint16_t bat_v_low : 1;         //����ϵͳǷѹ
            uint16_t can_receive_miss : 1;  //δ����CANͨ������
        } bit;
    }Cap_Status_t;

    /* �������ֵ */
    uint16_t Power_Available;      // �������ݷŵ�ӵ�ع���������

    /* ����ϵͳ��ֵ */
    uint16_t Power_Limit;          // ���ƹ���
    uint16_t PoweBuffer;           // ����ϵͳ�������Ĺ���ֵ
} PowerControl_Data_t;

typedef struct
{
    void (*PowerControl_Handle)(void);
	void (*PowerControl_MsgSend)(void);
	void (*PowerControl_MsgRec)(Fdcan_Export_Data_t RxMessage);
	void (*PowerContol_MsgRenew)(void);
} PowerControl_Fun_t;

extern PowerControl_Fun_t PowerControl_Fun;
extern PowerControl_Data_t PowerControl_Data;

#endif /*__POWERCONTROL_H*/
