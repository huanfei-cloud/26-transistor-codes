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

  M3508_Array[Fric_Front_1].targetSpeed = -Fric_Data.Required_Speed*0.7;
  M3508_Array[Fric_Front_2].targetSpeed = -Fric_Data.Required_Speed;
  M3508_Array[Fric_Front_3].targetSpeed = Fric_Data.Required_Speed*0.8;
  M3508_Array[Fric_Back_1].targetSpeed = -Fric_Data.Required_Speed*0.96*0.7;
  M3508_Array[Fric_Back_2].targetSpeed = -Fric_Data.Required_Speed*0.96;
  M3508_Array[Fric_Back_3].targetSpeed = Fric_Data.Required_Speed*0.96*0.8;

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
void Fric_Judge_ReadyOrNot()
{
  // static q15_t abs_differ[2] = {0};
  // static q15_t abs_factor[2];           //��Ŀ���ٶȵ�0.1Ϊ��
  // static q15_t temp[2];

  // temp[0] = Fric_Data.Required_Speed + M3508_Array[FricL_Wheel].realSpeed;
  // temp[1] = Fric_Data.Required_Speed - M3508_Array[FricR_Wheel].realSpeed;
  // arm_abs_q15(temp, abs_differ, 2);
  // temp[0] = Fric_Data.Required_Speed * 0.1f;
  // temp[1] = Fric_Data.Required_Speed * 0.1f;
  // arm_abs_q15(temp, abs_factor, 2);

  // if(abs_differ[0] < abs_factor[0] && abs_differ[1] < abs_factor[1])
  // 	Fric_Data.Fric_Ready = Fric_Ready;
  // else Fric_Data.Fric_Ready = Fric_NotReady;
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
    return;
  }
  else if (ControlMes.fric_Flag == 1)
  {
    Fric_Data.Required_Speed = Fric_SpeedLevel1;
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
