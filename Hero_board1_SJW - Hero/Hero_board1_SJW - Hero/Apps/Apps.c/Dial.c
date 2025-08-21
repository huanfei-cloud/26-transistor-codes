/**
 * @file Dial.c
 * @author Why,ZS,SJW
 * @brief 拨弹盘处理总函数
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "Dial.h"

/**************相关函数定义****************/
void Dial_Processing(void);
void Dial_Processing_1(void);
void Dial_Processing_2(void);
void PID_Clear_Dial(void);
float PID_Model4_Update(incrementalpid_t *pid, FUZZYPID_Data_t *PID, float _set_point, float _now_point);

#define CONTINUOUS_ROTATION_SPEED 50.0f //
#define CHECK_INTERVAL 1000             // 检测间隔
#define REVERSE_DURATION 500            // 反转时间
#define ANGLE_CHANGE_THRESHOLD 20       // 反转角度判断

/**************数据定义****************/
static uint32_t last_check_time = 0;    // 上一次检测时间
static int32_t last_angle = 0;          // 上一次角度
static uint8_t is_reversing = 0;        // 反转标志
static uint32_t reverse_start_time = 0; // 反转开始时间

/****************函数结构体声明******************/
Dial_Fun_t Dial_Fun = Dial_FunGroundInit;
#undef Dial_FunGroundInit
Dial_Data_t Dial_Data = Dial_DataGroundInit;
#undef Dial_DataGroundInit
/**
 * @brief  拨弹处理函数
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing(void)
{
  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M3508_Array[Dial_Wheel].targetSpeed = -1000; // Dial_Data.Speed_Dial;

    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
  }
  else
  {
    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, 0, M3508_Array[Dial_Wheel].realSpeed);
  }
}

/**
 * @brief  拨弹处理函数1
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_1(void)
{

  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M3508_Array[Dial_Wheel].targetAngle -= 5;

    M3508_Array[Dial_Wheel].targetSpeed = Position_PID(&M3508_DialV_Pid, M3508_Array[Dial_Wheel].targetAngle, M3508_Array[Dial_Wheel].totalAngle);

    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);

    /********************检测堵转*****************/
    if (M3508_Array[Dial_Wheel].realCurrent > 10000 || M3508_Array[Dial_Wheel].realCurrent < -10000)
    {
      M3508_Array[Dial_Wheel].targetAngle += 1000;

      M3508_Array[Dial_Wheel].targetSpeed = Position_PID(&M3508_DialV_Pid, M3508_Array[Dial_Wheel].targetAngle, M3508_Array[Dial_Wheel].totalAngle);

      M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
    }
  }
  else
  {
    M3508_Array[Dial_Wheel].targetSpeed = 0;
    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, 0, M3508_Array[Dial_Wheel].realSpeed);
  }
}

/**
 * @brief  拨弹处理函数2
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_2(void)
{
  uint32_t current_time = HAL_GetTick(); // 获取当前时间

  // 如果正在反向
  if (is_reversing)
  {
    // 检测时间是否达到一秒
    if (current_time - reverse_start_time >= REVERSE_DURATION)
    {
      // 取消反转
      is_reversing = 0;
    }
    else
    {
      // 不足一秒
      M3508_Array[Dial_Wheel].targetSpeed = 1000; // Dial_Data.Speed_Dial;
      M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
      return; // 退出函数
    }
  }

  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    M3508_Array[Dial_Wheel].targetSpeed = -1000; // Dial_Data.Speed_Dial;

    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);

    /********************拨弹盘卡弹处理*****************/
    // 定时检测角度变化(每CHECK_INTERVAL ms检测一次)
    if (current_time - last_check_time >= CHECK_INTERVAL)
    {
      // 计算角度变化量(取绝对值)
      int32_t angle_change = abs(M3508_Array[Dial_Wheel].totalAngle - last_angle);

      // 如果角度变化小于阈值，判断为卡弹
      if (angle_change < ANGLE_CHANGE_THRESHOLD)
      {
        is_reversing = 1;
        reverse_start_time = current_time;
        // 立即开始反转
        M3508_Array[Dial_Wheel].targetSpeed = 1000;
        M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
      }

      // 更新检测基准值
      last_check_time = current_time;
      last_angle = M3508_Array[Dial_Wheel].totalAngle;
    }
  }

  /*******************************应被注释的内容**************************************/
  // static bool is_dial_rotating = false;
  // static float target_rotate_angle = 0.0f;

  // if (Dial_Data.Shoot_Mode == Single_Shoot && Dial_Data.Dial_Switch == Dial_Single && !is_dial_rotating)
  // {
  //   // 开始转动：设置目标角度为当前角度+60度，标记为正在转动
  //   target_rotate_angle = M3508_Array[Dial_Wheel].totalAngle + 60.0f;
  //   is_dial_rotating = true;
  // }
  // // 正在转动过程中，进行角度闭环控制
  // if (is_dial_rotating)
  // {
  //   // 通过PID计算目标速度（根据角度差控制）
  //   M3508_Array[Dial_Wheel].targetSpeed = PID_Model4_Update(&M3508_DialV_Pid, &fuzzy_pid_bullet_v, target_rotate_angle, M3508_Array[Dial_Wheel].totalAngle);

  //   // 计算输出电流控制电机
  //   M3508_Array[Dial_Wheel].outCurrent = Incremental_PID(&M3508_DialI_Pid, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);

  //   // 检查是否已转动到目标角度（允许±2度误差）
  //   if (fabs(M3508_Array[Dial_Wheel].totalAngle - target_rotate_angle) <= 2.0f)
  //   {
  //     // 转动完成：停止电机，关闭拨盘开关，重置状态
  //     M3508_Array[Dial_Wheel].outCurrent = 0;
  //     Dial_Data.Dial_Switch = Dial_Off; // 关闭拨盘开关
  //     is_dial_rotating = false;
  //     target_rotate_angle = 0.0f;
  //   }
  // }
  // 拨弹盘状态标志和目标角度（静态变量，保持状态）

  /**********************************************************************************/
  static bool dial_rotation_finish = false;    // 转动是否完成
  static float target_rotate_angle = 0.0f; // 目标转动角度（当前角度+60度）
  static uint16_t Single_Switch = 0;       //单发开关
  //单发开关判断逻辑
  if (Dial_Data.Shoot_Mode == No_Shoot) dial_rotation_finish = false;//s2拨回到MID时，刷新转动完成标志到false

  if (dial_rotation_finish == false && Dial_Data.Shoot_Mode == Single_Shoot) Single_Switch = 1;
  else Single_Switch = 0;

  // 主逻辑：整合判断条件，用嵌套结构处理不同状态
  // 启动条件：处于单发模式，且未完成一次转动
  if (Single_Switch == 1)
  {
    // 初始化转动参数：目标角度为当前角度+60度，执行转动控制
    target_rotate_angle = M3508_Array[Dial_Wheel].totalAngle + 60.0f;
    // 角度闭环控制：计算目标速度（PID调节）
    M3508_Array[Dial_Wheel].targetSpeed = Position_PID(&M3508_DialV_Pid, target_rotate_angle, M3508_Array[Dial_Wheel].totalAngle);
    // 电流闭环控制（根据目标速度调节输出）
    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, M3508_Array[Dial_Wheel].targetSpeed, M3508_Array[Dial_Wheel].realSpeed);
    // 检查是否到达目标角度（允许±2度误差，避免抖动） //若形成±2°以上的超调，则会在检测时停止电机、关闭拨盘开关、重置状态，并由惯性继续转动，因此PID调参需注意注意避免超调。
    if (fabs(M3508_Array[Dial_Wheel].totalAngle - target_rotate_angle) <= 2.0f)
    {
      // 转动完成：停止电机、关闭拨盘开关、重置状态
      M3508_Array[Dial_Wheel].outCurrent = 0; // 停止输出
      dial_rotation_finish = true;            // 转动完成标志
      target_rotate_angle = 0.0f;             // 清空目标角度
    }
  }
  else
  {
    M3508_Array[Dial_Wheel].outCurrent = PID_Model4_Update(&M3508_DialI_Pid, &fuzzy_pid_bullet_v, 0, M3508_Array[Dial_Wheel].realSpeed);
  }
}

/**
 * @brief  PID重置
 * @retval void
 * @attention
 */
void PID_Clear_Dial(void)
{

  Position_PIDInit(&M3508_DialV_Pid, 0.3f, 0.001f, 0.4, 0.5, 20000, 8000, 700); // 拨盘电机速度环
  Incremental_PIDInit(&M3508_DialI_Pid, 12.5f, 0.5f, 0, 10000, 1000);           // 拨盘电机电流环
}
