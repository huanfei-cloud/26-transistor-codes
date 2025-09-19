/**
 * @file Dial.c
 * @author Why,ZS,SJW
 * @brief 处理拨盘电机的控制问题
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Dial.h"
#include "FuzzyPID.h"
#include "PID.h"
#include "FeedForward.h"
#include "stm32h7xx_hal.h"

/**************用户数据定义****************/
void Dial_Processing(void);
void Dial_Processing_1(void);
void Dial_Processing_2(void);
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void PID_Clear_Dial(void);
float PID_Model4_Update(incrementalpid_t *pid, FUZZYPID_Data_t *PID, float _set_point, float _now_point);

#define CONTINUOUS_ROTATION_SPEED 50.0f //
#define CHECK_INTERVAL          1000    // 角度检测间隔(ms)
#define REVERSE_DURATION        500    // 反转持续时间(ms)
#define ANGLE_CHANGE_THRESHOLD  10      // 角度变化阈值(判断是否卡弹)

/**************静态变量用于卡弹检测****************/
static uint32_t last_check_time = 0;    // 上次检测时间
static int32_t last_angle = 0;          // 上次检测时的角度
static uint8_t is_reversing = 0;        // 反转状态标记
static uint32_t reverse_start_time = 0; // 反转开始时间



/****************接口定义******************/
Dial_Fun_t Dial_Fun = Dial_FunGroundInit;
#undef Dial_FunGroundInit
Dial_Data_t Dial_Data = Dial_DataGroundInit;
#undef Dial_DataGroundInit
/**
 * @brief  拨盘电机总处理函数
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing(void)
{
  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M2006_Array[Dial_Motor].targetSpeed = -1000; // Dial_Data.Speed_Dial;

    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);

    /********************拨弹盘卡弹处理*****************/

    /*    if (M2006_Array[Dial_Motor].realTorque > 9200 || M2006_Array[Dial_Motor].realTorque < -9200)
        {
          M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 500, M2006_Array[Dial_Motor].realSpeed);
        }*/
  }
  else
  {
    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 0, M2006_Array[Dial_Motor].realSpeed);
  }
}

/**
 * @brief  拨盘电机总处理函数1(角度环)
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_1(void)
{

  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M2006_Array[Dial_Motor].targetAngle -= 5;

    M2006_Array[Dial_Motor].targetSpeed = Position_PID(&M2006_DialV_Pid, M2006_Array[Dial_Motor].targetAngle, M2006_Array[Dial_Motor].totalAngle);

    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);

    /********************拨弹盘卡弹处理*****************/
    if (M2006_Array[Dial_Motor].realTorque > 10000 || M2006_Array[Dial_Motor].realTorque < -10000)
    {
      M2006_Array[Dial_Motor].targetAngle += 1000;

      M2006_Array[Dial_Motor].targetSpeed = Position_PID(&M2006_DialV_Pid, M2006_Array[Dial_Motor].targetAngle, M2006_Array[Dial_Motor].totalAngle);

      M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
    }
  }
  else
  {
    M2006_Array[Dial_Motor].targetSpeed = 0;
    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 0, M2006_Array[Dial_Motor].realSpeed);
  }
}

/**
 * @brief  拨盘电机总处理函数（速度环）
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_2(void)
{
  uint32_t current_time = HAL_GetTick(); // 获取系统当前时间(ms)

  // 如果正在反转状态
  if (is_reversing)
  {
    // 检查反转是否已持续足够时间
    if (current_time - reverse_start_time >= REVERSE_DURATION)
    {
        // 结束反转
        is_reversing = 0;
    }
    else
    {
        // 执行反转：设置反转速度
        M2006_Array[Dial_Motor].targetSpeed = 1000; // Dial_Data.Speed_Dial;
        M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
        return; // 退出函数，不执行后续逻辑
    }
  }

  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M2006_Array[Dial_Motor].targetSpeed = -1000; // Dial_Data.Speed_Dial;

    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);

    /********************拨弹盘卡弹处理*****************/
    // 定时检测角度变化(每CHECK_INTERVAL ms检测一次)
    if (current_time - last_check_time >= CHECK_INTERVAL)
    {
      // 计算角度变化量(取绝对值)
      int32_t angle_change = abs(M2006_Array[Dial_Motor].totalAngle - last_angle);
            
      // 如果角度变化小于阈值，判断为卡弹
      if (angle_change < ANGLE_CHANGE_THRESHOLD)
      {
        is_reversing = 1;
        reverse_start_time = current_time;
        // 立即开始反转
        M2006_Array[Dial_Motor].targetSpeed = 1000;
        M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed,  M2006_Array[Dial_Motor].realSpeed);
      }
            
      // 更新检测基准值
      last_check_time = current_time;
      last_angle = M2006_Array[Dial_Motor].totalAngle;
    }
  }
  else
  {
    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 0, M2006_Array[Dial_Motor].realSpeed);
  }
}

/**
 * @brief  更新拨盘电机的角度值
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Update_Angel(bool Fric_ReadyOrNot)
{
}

/**
 * @brief  Dial电机PID清除
 * @param  void
 * @retval void
 * @attention
 */
void PID_Clear_Dial(void)
{

  Position_PIDInit(&M2006_DialV_Pid, 0.3f, 0.001f, 0.4, 0.5, 20000, 8000, 700); // 拨盘电机速度环
  Incremental_PIDInit(&M2006_DialI_Pid, 12.5f, 0.5f, 0, 10000, 1000);           // 拨盘电机电流环
}
