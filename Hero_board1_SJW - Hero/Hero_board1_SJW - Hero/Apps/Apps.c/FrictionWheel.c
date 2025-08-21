/**
 * @file FrictionWheel.c
 * @author Why
 * @brief 处理摩擦轮的控制问题
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "FrictionWheel.h"

/***************变量定义****************/
// 添加全局变量用于记录摩擦轮启动时间和状态
static uint32_t fric_start_time = 0;
static uint8_t fric_ramp_up_active = 0;
static float fric_target_speed = 0;
static uint8_t fric_already_started = 0;

/**************用户数据定义****************/
void Fric_Processing(void);
void Fric_Judge_ReadyOrNot(void);
void Fric_Set_targetSpeed(void);
void PID_Clear_FricL(void);
void PID_Clear_FricR(void);
float PID_Model4_Update(incrementalpid_t *pid, FUZZYPID_Data_t *PID, float _set_point, float _now_point);

/****************接口定义******************/
Fric_Fun_t Fric_Fun = Fric_FunGroundInit;
#undef Fric_FunGroundInit
Fric_Data_t Fric_Data = Fric_DataGroundInit;
#undef Fric_DataGroundInit

/**
 * @brief  摩擦轮控制总处理函数
 * @param  void
 * @retval void
 * @attention
 */
void Fric_Processing()
{
  /* 设定目标值 */
  Fric_Set_targetSpeed();

  static float current_speed = 0;
  uint32_t current_time = HAL_GetTick();

  // 检查是否处于2秒加速过程中
  if (fric_ramp_up_active)
  {
    uint32_t elapsed_time = current_time - fric_start_time;

    // 如果已经过了2秒，结束加速过程
    if (elapsed_time >= 2000)
    {
      current_speed = fric_target_speed;
      fric_ramp_up_active = 0;
    }
    else
    {
      // 在2秒内从目标速度的一半增加到目标速度
      // 使用线性插值: speed = initial_speed + (target_speed - initial_speed) * (elapsed_time / 2000)
      float initial_speed = fric_target_speed / 2.0f;
      current_speed = initial_speed + (fric_target_speed - initial_speed) * ((float)elapsed_time / 2000.0f);
    }
  }
  else
  {
    // 正常情况下使用目标速度
    current_speed = fric_target_speed;
  }

  // 应用当前速度到各个电机
  if (current_speed > 0)
  {
    M3508_Array[Fric_Front_1].targetSpeed = -current_speed * 0.7f;
    M3508_Array[Fric_Front_2].targetSpeed = -current_speed;
    M3508_Array[Fric_Front_3].targetSpeed = current_speed * 0.8f;
    M3508_Array[Fric_Back_1].targetSpeed = -current_speed * 0.96f * 0.7f;
    M3508_Array[Fric_Back_2].targetSpeed = -current_speed * 0.96f;
    M3508_Array[Fric_Back_3].targetSpeed = current_speed * 0.96f * 0.8f;
  }
  else
  {
    // 如果速度为0，停止所有电机
    M3508_Array[Fric_Front_1].targetSpeed = 0;
    M3508_Array[Fric_Front_2].targetSpeed = 0;
    M3508_Array[Fric_Front_3].targetSpeed = 0;
    M3508_Array[Fric_Back_1].targetSpeed = 0;
    M3508_Array[Fric_Back_2].targetSpeed = 0;
    M3508_Array[Fric_Back_3].targetSpeed = 0;
  }

  /************PID计算************/
  M3508_Array[Fric_Front_1].outCurrent = PID_Model4_Update(&M3508_FricF1_Pid, &fuzzy_pid_shoot_F1, M3508_Array[Fric_Front_1].targetSpeed, M3508_Array[Fric_Front_1].realSpeed);
  M3508_Array[Fric_Front_2].outCurrent = PID_Model4_Update(&M3508_FricF2_Pid, &fuzzy_pid_shoot_F2, M3508_Array[Fric_Front_2].targetSpeed, M3508_Array[Fric_Front_2].realSpeed);
  M3508_Array[Fric_Front_3].outCurrent = PID_Model4_Update(&M3508_FricF3_Pid, &fuzzy_pid_shoot_F3, M3508_Array[Fric_Front_3].targetSpeed, M3508_Array[Fric_Front_3].realSpeed);
  M3508_Array[Fric_Back_1].outCurrent = PID_Model4_Update(&M3508_FricB1_Pid, &fuzzy_pid_shoot_B1, M3508_Array[Fric_Back_1].targetSpeed, M3508_Array[Fric_Back_1].realSpeed);
  M3508_Array[Fric_Back_2].outCurrent = PID_Model4_Update(&M3508_FricB2_Pid, &fuzzy_pid_shoot_B2, M3508_Array[Fric_Back_2].targetSpeed, M3508_Array[Fric_Back_2].realSpeed);
  M3508_Array[Fric_Back_3].outCurrent = PID_Model4_Update(&M3508_FricB3_Pid, &fuzzy_pid_shoot_B3, M3508_Array[Fric_Back_3].targetSpeed, M3508_Array[Fric_Back_3].realSpeed);
}

/**
 * @brief  当摩擦轮的速度达到target附近时认为摩擦轮已经就绪
 * @param  void
 * @retval void
 * @attention  这个函数未启用是因为H板代码arm_abs_q15等arm函数会报错
 */
// void Fric_Judge_ReadyOrNot()
// {
//   static q15_t abs_differ[2] = {0};
//   static q15_t abs_factor[2];
//   static q15_t temp[2];

//   temp[0] = Fric_Data.Required_Speed + M3508_Array[FricL_Wheel].realSpeed;
//   temp[1] = Fric_Data.Required_Speed - M3508_Array[FricR_Wheel].realSpeed;
//   arm_abs_q15(temp, abs_differ, 2);
//   temp[0] = Fric_Data.Required_Speed * 0.1f;
//   temp[1] = Fric_Data.Required_Speed * 0.1f;
//   arm_abs_q15(temp, abs_factor, 2);

//   if(abs_differ[0] < abs_factor[0] && abs_differ[1] < abs_factor[1])
//   	Fric_Data.Fric_Ready = Fric_Ready;
//   else Fric_Data.Fric_Ready = Fric_NotReady;
// }
void Fric_Judge_ReadyOrNot()
{
  // 计算所有6个摩擦轮实际速度与目标速度的差值
  float diff[6];
  float tolerance = Fric_Data.Required_Speed * 0.1f; // 10%的容差

  diff[0] = M3508_Array[Fric_Front_1].targetSpeed - M3508_Array[Fric_Front_1].realSpeed;
  diff[1] = M3508_Array[Fric_Front_2].targetSpeed - M3508_Array[Fric_Front_2].realSpeed;
  diff[2] = M3508_Array[Fric_Front_3].targetSpeed - M3508_Array[Fric_Front_3].realSpeed;
  diff[3] = M3508_Array[Fric_Back_1].targetSpeed - M3508_Array[Fric_Back_1].realSpeed;
  diff[4] = M3508_Array[Fric_Back_2].targetSpeed - M3508_Array[Fric_Back_2].realSpeed;
  diff[5] = M3508_Array[Fric_Back_3].targetSpeed - M3508_Array[Fric_Back_3].realSpeed;

  // 检查所有摩擦轮是否都在容差范围内
  if ((diff[0] < tolerance && diff[0] > -tolerance) &&
      (diff[1] < tolerance && diff[1] > -tolerance) &&
      (diff[2] < tolerance && diff[2] > -tolerance) &&
      (diff[3] < tolerance && diff[3] > -tolerance) &&
      (diff[4] < tolerance && diff[4] > -tolerance) &&
      (diff[5] < tolerance && diff[5] > -tolerance))
    Fric_Data.Fric_Ready = Fric_Ready;
  else
    Fric_Data.Fric_Ready = Fric_NotReady;
}

/**
 * @brief  设定摩擦轮的目标速度
 * @param  void
 * @retval void
 * @attention
 */
void Fric_Set_targetSpeed(void)
{

  if (ControlMes.fric_Flag == 0)
  {
    Fric_Data.Required_Speed = 0;
  }
  else if (ControlMes.fric_Flag == 1)
  {
    Fric_Data.Required_Speed = Fric_SpeedLevel1;
  }
  // 记录目标速度
  fric_target_speed = Fric_Data.Required_Speed;

  // 如果是第一次启动摩擦轮，开始2秒加速过程
  if (!fric_already_started && Fric_Data.Required_Speed > 0)
  {
    fric_start_time = HAL_GetTick(); // 获取当前系统时间
    fric_ramp_up_active = 1;
    fric_already_started = 1; // 标记已经启动过
  }
  else if (Fric_Data.Required_Speed <= 0)
  {
    // 如果关闭摩擦轮，重置状态
    fric_ramp_up_active = 0;
    fric_target_speed = 0;
    fric_already_started = 0; // 重置启动标记，以便下次重新启动时再次使用加速
  }
}

/**
 * @brief  左摩擦轮的PID重置
 * @param  void
 * @retval void
 * @attention//75 0.03 05 //3.0 0.03 0
 */
void PID_Clear_FricL(void)
{
  // Incremental_PIDInit(&M3508_FricL_Pid, 3.3f, 0.035f, 0, 8000, 700);
}

/**
 * @brief  右摩擦轮的PID重置
 * @param  void
 * @retval void
 * @attention
 */
void PID_Clear_FricR(void)
{
  // Incremental_PIDInit(&M3508_FricR_Pid, 3.3f, 0.035f, 0, 8000, 700);
}
