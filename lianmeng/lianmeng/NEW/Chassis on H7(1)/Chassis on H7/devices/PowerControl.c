/**
 * @file  PowerControl.c
 * @author Why
 * @brief ����������������ƹ���֮��
 * @version 0.2
 * @date 2024-1-18
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "PowerControl.h"
#include "motor.h"
#include "bsp_fdcan.h"
#include "arm_math.h"
#include "chassis.h"

/***************�û���������****************/
void PowerControl_Handle(void);
void PowerControl_Update(void);
void PowerControl_Cal_Power(void);
void PowerControl_Cal_Torque(void);
void PowerControl_MsgSend(void);
void PowerControl_MsgRec(struct Struct_CAN_Rx_Buffer RxMessage);

/* ������ת��Ϊ������DSP��FPU�� */
static float TC[4] = {Torque_Constant, Torque_Constant,
                      Torque_Constant, Torque_Constant};
static float MC[4] = {MachinePower_Cnstant, MachinePower_Cnstant,
                      MachinePower_Cnstant, MachinePower_Cnstant};
static float IC[4] = {It_Constant, It_Constant, It_Constant, It_Constant};
static float Kw[4] = {Kw_d, Kw_d, Kw_d, Kw_d};
static float Kt[4] = {Kt_d, Kt_d, Kt_d, Kt_d};
static float A[4] = {A_d, A_d, A_d, A_d};
static float Div_2Kt[4] = {Div_2Kt_d, Div_2Kt_d, Div_2Kt_d, Div_2Kt_d};

/* �����˻�е���ʡ�����ͭ�����ء�w����9.55��ֵ */
static float Pm_Cal[4], Kw2[4], Kt2[4], t_cal[4], w_MC[4];
/* ʵ�ʸ������ӹ��� */
static float Pin[4];

/* ��ԭ�ṹ���е�����������͵��������DSP���� */
static float I_Array[4], Speed_Array[4];

/******************�ӿ�����*****************/
PowerControl_Fun_t PowerControl_Fun = PowerControl_FunGroundInit;
#undef PowerControl_FunGroundInit
PowerControl_Data_t PowerControl_Data = PowerControl_DataGroundInit;
#undef PowerControl_DataGroundInit

/**
 * @brief  16λ����ת��Ϊ�����������㷢��Ҫ��
 * @param
 * @retval
 */
static float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max,
                            float b_min)
{
  float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
  return b;
}

/**
  * @brief  ���������Ƶĺ����߼�����,���ڵ����ĸ�����
  * @param  extern ���PID�ṹ�������׵�ַ
            extern ������ݽṹ�������׵�ַ�ṹ��
            extern �������ݿ��ṩ��������
            *aim_address[] ��ʵ�ʵ��ת�ص���ֵ�ĵ�ַ
  * @retval �Ѵ��ַ��
  */
void PowerControl_Handle()
{
  static float Pin_all, Scale[4];
  PowerControl_Update();
  PowerControl_Cal_Power();
  Pin_all = Pin[0] + Pin[1] + Pin[2] + Pin[3];

  if (Pin_all <= PowerControl_Data.Power_Available)
    return;
  else
  {
    Scale[0] = PowerControl_Data.Power_Available / Pin_all;
    Scale[1] = Scale[0];
    Scale[2] = Scale[0];
    Scale[3] = Scale[0];
    arm_mult_f32(Pin, Scale, Pin, 4);
    PowerControl_Cal_Torque();
  }
}

/**
 * @brief  ���㵱ǰת�ٺ�ת�ص����µĹ���
 * @param
 * @retval
 * @attention �õ�ת���ǵ�ǰת�٣�������Ԥ�ڵ�������Ҫ������ת�ص�������˲��ﵽ
 */
void PowerControl_Cal_Power()
{
  static float exp1[4], exp2[4]; // �м����
  /* �ȼ��������ǰ������ */
  arm_mult_f32(TC, I_Array, t_cal, 4);
  /* �����е���� */
  arm_mult_f32(Speed_Array, MC, w_MC, 4);
  arm_mult_f32(w_MC, t_cal, Pm_Cal, 4);
  /* ����w�ͦ���൱�������ͭ�� */
  arm_mult_f32(Speed_Array, Speed_Array, exp1, 4);
  arm_mult_f32(exp1, Kw, Kw2, 4);

  arm_mult_f32(t_cal, t_cal, exp1, 4);
  arm_mult_f32(exp1, Kt, Kt2, 4);

  /* �������ڵ�ʵ�ʹ��� */
  arm_add_f32(Pm_Cal, Kw2, exp1, 4);
  arm_add_f32(Kt2, exp1, exp2, 4);
  arm_add_f32(A, exp2, Pin, 4);
}

/**
 * @brief  ���㵱ǰת�ٺ��趨�����µ�ת�ص���
 * @param
 * @retval
 */
void PowerControl_Cal_Torque()
{
  /* �����м�ֵ */
  static float exp1[4], exp2[4], exp3[4];
  static float Array_4[4] = {4, 4, 4, 4};
  static uint8_t i;

  /* �����ʽ t = (-w/9.55 �� sqrt((w/9.55)^2 - 4*Kt(Kw*w^2 + A - Pin)))/(2*Kt) */
  arm_add_f32(Kw2, A, exp1, 4);    // Kw*w^2 + A
  arm_sub_f32(exp1, Pin, exp2, 4); // Kw*w^2 + A - Pin

  arm_mult_f32(Array_4, Kt, exp1, 4); // 4*Kt
  arm_mult_f32(exp1, exp2, exp3, 4);  // 4*Kt(Kw*w^2 + A - Pin)

  arm_mult_f32(w_MC, w_MC, exp1, 4); //(w/9.55)^2
  arm_sub_f32(exp1, exp3, exp2, 4);  //(w/9.55)^2 - 4*Kt(Kw*w^2 + A - Pin)

  /* sqrt((w/9.55)^2 - 4*Kt(Kw*w^2 + A - Pin)) */
  for (i = 0; i <= 3; i++)
    arm_sqrt_f32(exp2[i], &exp1[i]);
  arm_negate_f32(w_MC, exp2, 4); //-w/9.55

  for (i = 0; i <= 3; i++)
  {
    if (I_Array[i] <= 0)
      arm_sub_f32(&exp2[i], &exp1[i], &exp3[i], 1);
    else
      arm_add_f32(&exp2[i], &exp1[i], &exp3[i], 1);
  }

  arm_mult_f32(exp3, Div_2Kt, exp1, 4);
  arm_mult_f32(exp1, IC, exp2, 4);

  // ����Ҫ����˳��
  //		chassis_control.motor1.v_pid_object.output=(int16_t)exp2[0];
  //		chassis_control.motor2.v_pid_object.output=(int16_t)exp2[1];
  //		chassis_control.motor3.v_pid_object.output=(int16_t)exp2[2];
  //		chassis_control.motor4.v_pid_object.output=(int16_t)exp2[3];
}

/**
 * @brief  ����ת����ת�ص��������ʿ��ƵĽṹ����
 * @param
 * @retval void
 */
void PowerControl_Update()
{

  I_Array[0] = (float)chassis_control.motor1.v_pid_object.output;
  Speed_Array[0] = (float)chassis_control.motor1.omega;
  I_Array[1] = (float)chassis_control.motor2.v_pid_object.output;
  Speed_Array[1] = (float)chassis_control.motor2.omega;
  I_Array[2] = (float)chassis_control.motor3.v_pid_object.output;
  Speed_Array[2] = (float)chassis_control.motor3.omega;
  I_Array[3] = (float)chassis_control.motor4.v_pid_object.output;
  Speed_Array[3] = (float)chassis_control.motor4.omega;
}

/**
 * @brief  �������ݵ��������ݿ��ư�
 * @param
 * @retval void
 */
void PowerControl_MsgSend()
{
  static uint8_t time;
  static uint8_t data[8];
  /* ���Ʒ��͵�Ƶ��,���㹦�ʿ��ư������ */
  if (time++ <= 20)
    return;
  time = 0;

  /* ����0x2E��Ӧ�Ļ��幦������ */
  memset(data, 0, 8);
  data[0] = PowerControl_Data.PoweBuffer >> 8;
  data[1] = PowerControl_Data.PoweBuffer;
  fdcanx_send_data(&hfdcan1, PowerBuffer_ID, data, 8);

  /* ����0x2F��Ӧ�ĵ��̹����������� */
  data[0] = PowerControl_Data.Power_Limit >> 8;
  data[1] = PowerControl_Data.Power_Limit;
  data[2] = PowerControl_Data.DischargeLimit >> 8;
  data[3] = PowerControl_Data.DischargeLimit;
  data[4] = PowerControl_Data.ChargeLimit >> 8;
  data[5] = PowerControl_Data.ChargeLimit;
  data[6] = PowerControl_Data.ControlBit >> 8;
  data[7] = PowerControl_Data.ControlBit;
  fdcanx_send_data(&hfdcan1, PowerLimit_ID, data, 8);
}

/**
 * @brief  ���㳬�����ݿ��ư巢�ص�����
 * @param
 * @retval void
 */
void PowerControl_MsgRec(struct Struct_CAN_Rx_Buffer RxMessage)
{
  static int16_t data;
  // ������ݣ����ݸ�ʽ����������ݵļ���˵����
  data = (int16_t)(RxMessage.Data[0] << 8 | RxMessage.Data[1]);
  PowerControl_Data.Cap_U = int16_to_float(data, 32000, -32000, 30, 0);
  data = (int16_t)(RxMessage.Data[2] << 8 | RxMessage.Data[3]);
  PowerControl_Data.Cap_I = int16_to_float(data, 32000, -32000, 20, -20);
  data = (uint16_t)(RxMessage.Data[4] << 8 | RxMessage.Data[5]);
  PowerControl_Data.Cap_Status_t.Cap_Status = data;
}
