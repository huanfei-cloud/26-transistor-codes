/**
 * @file  PowerControl.c
 * @author Why
 * @brief 控制输出功率在限制功率之下
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

/***************用户数据声明****************/
void PowerControl_Handle(void);
void PowerControl_Update(void);
void PowerControl_Cal_Power(void);
void PowerControl_Cal_Torque(void);
void PowerControl_MsgSend(void);
void PowerControl_MsgRec(struct Struct_CAN_Rx_Buffer RxMessage);

/* 将常数转化为数组让DSP用FPU算 */
static float TC[4] = {Torque_Constant, Torque_Constant,
                      Torque_Constant, Torque_Constant};
static float MC[4] = {MachinePower_Cnstant, MachinePower_Cnstant,
                      MachinePower_Cnstant, MachinePower_Cnstant};
static float IC[4] = {It_Constant, It_Constant, It_Constant, It_Constant};
static float Kw[4] = {Kw_d, Kw_d, Kw_d, Kw_d};
static float Kt[4] = {Kt_d, Kt_d, Kt_d, Kt_d};
static float A[4] = {A_d, A_d, A_d, A_d};
static float Div_2Kt[4] = {Div_2Kt_d, Div_2Kt_d, Div_2Kt_d, Div_2Kt_d};

/* 定义了机械功率、铁损、铜损、力矩、w除以9.55的值 */
static float Pm_Cal[4], Kw2[4], Kt2[4], t_cal[4], w_MC[4];
/* 实际各个轮子功率 */
static float Pin[4];

/* 将原结构体中的数存进浮点型的数组便于DSP运算 */
static float I_Array[4], Speed_Array[4];

/******************接口声明*****************/
PowerControl_Fun_t PowerControl_Fun = PowerControl_FunGroundInit;
#undef PowerControl_FunGroundInit
PowerControl_Data_t PowerControl_Data = PowerControl_DataGroundInit;
#undef PowerControl_DataGroundInit

/**
 * @brief  16位整数转化为浮点数，满足发送要求
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
  * @brief  处理功率限制的核心逻辑函数,限于底盘四个轮子
  * @param  extern 电机PID结构体数据首地址
            extern 电机数据结构体数据首地址结构体
            extern 超级电容可提供功率上限
            *aim_address[] 存实际电机转矩电流值的地址
  * @retval 已存地址中
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
 * @brief  计算当前转速和转矩电流下的功率
 * @param
 * @retval
 * @attention 用的转速是当前转速，电流是预期电流，重要假设是转矩电流可以瞬间达到
 */
void PowerControl_Cal_Power()
{
  static float exp1[4], exp2[4]; // 中间变量
  /* 先计算减速箱前的力矩 */
  arm_mult_f32(TC, I_Array, t_cal, 4);
  /* 计算机械功率 */
  arm_mult_f32(Speed_Array, MC, w_MC, 4);
  arm_mult_f32(w_MC, t_cal, Pm_Cal, 4);
  /* 计算w和τ项，相当于铁损和铜损 */
  arm_mult_f32(Speed_Array, Speed_Array, exp1, 4);
  arm_mult_f32(exp1, Kw, Kw2, 4);

  arm_mult_f32(t_cal, t_cal, exp1, 4);
  arm_mult_f32(exp1, Kt, Kt2, 4);

  /* 计算现在的实际功率 */
  arm_add_f32(Pm_Cal, Kw2, exp1, 4);
  arm_add_f32(Kt2, exp1, exp2, 4);
  arm_add_f32(A, exp2, Pin, 4);
}

/**
 * @brief  计算当前转速和设定功率下的转矩电流
 * @param
 * @retval
 */
void PowerControl_Cal_Torque()
{
  /* 定义中间值 */
  static float exp1[4], exp2[4], exp3[4];
  static float Array_4[4] = {4, 4, 4, 4};
  static uint8_t i;

  /* 求根公式 t = (-w/9.55 ± sqrt((w/9.55)^2 - 4*Kt(Kw*w^2 + A - Pin)))/(2*Kt) */
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

  // 这里要调整顺序
  //		chassis_control.motor1.v_pid_object.output=(int16_t)exp2[0];
  //		chassis_control.motor2.v_pid_object.output=(int16_t)exp2[1];
  //		chassis_control.motor3.v_pid_object.output=(int16_t)exp2[2];
  //		chassis_control.motor4.v_pid_object.output=(int16_t)exp2[3];
}

/**
 * @brief  更新转速与转矩电流到功率控制的结构体中
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
 * @brief  发送数据到超级电容控制板
 * @param
 * @retval void
 */
void PowerControl_MsgSend()
{
  static uint8_t time;
  static uint8_t data[8];
  /* 限制发送的频率,满足功率控制板的需求 */
  if (time++ <= 20)
    return;
  time = 0;

  /* 发送0x2E对应的缓冲功率数据 */
  memset(data, 0, 8);
  data[0] = PowerControl_Data.PoweBuffer >> 8;
  data[1] = PowerControl_Data.PoweBuffer;
  fdcanx_send_data(&hfdcan1, PowerBuffer_ID, data, 8);

  /* 发送0x2F对应的底盘功率限制数据 */
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
 * @brief  解算超级电容控制板发回的数据
 * @param
 * @retval void
 */
void PowerControl_MsgRec(struct Struct_CAN_Rx_Buffer RxMessage)
{
  static int16_t data;
  // 解包数据，数据格式详见超级电容的技术说明书
  data = (int16_t)(RxMessage.Data[0] << 8 | RxMessage.Data[1]);
  PowerControl_Data.Cap_U = int16_to_float(data, 32000, -32000, 30, 0);
  data = (int16_t)(RxMessage.Data[2] << 8 | RxMessage.Data[3]);
  PowerControl_Data.Cap_I = int16_to_float(data, 32000, -32000, 20, -20);
  data = (uint16_t)(RxMessage.Data[4] << 8 | RxMessage.Data[5]);
  PowerControl_Data.Cap_Status_t.Cap_Status = data;
}
