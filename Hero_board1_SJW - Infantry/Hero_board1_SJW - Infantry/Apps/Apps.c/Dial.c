/**
 * @file Dial.c
 * @author Why,ZS,SJW,ZTC
 * @brief 拨弹盘函数
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
void Dial_Update_Angel(bool Fric_ReadyOrNot);
void PID_Clear_Dial(void);
float PID_Model4_Update(incrementalpid_t *pid, FUZZYPID_Data_t *PID, float _set_point, float _now_point);

#define CONTINUOUS_ROTATION_SPEED 50.0f //
#define CHECK_INTERVAL 800              // 检测间隔
#define REVERSE_DURATION 500            // 反转时间
#define ANGLE_CHANGE_THRESHOLD 20       // 

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
Heat_Data_t Heat_Data = Heat_Data_Init_Burst_First_Level1;//因为当前无法获取优先级模式，因此暂时默认为爆发优先
#undef Heat_Data_Init_Burst_First_Level1

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
    M2006_Array[Dial_Motor].targetSpeed = -1000; // Dial_Data.Speed_Dial;

    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
  }
  else
  {
    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 0, M2006_Array[Dial_Motor].realSpeed);
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
    M2006_Array[Dial_Motor].targetAngle -= 5;

    M2006_Array[Dial_Motor].targetSpeed = Position_PID(&M2006_DialV_Pid, M2006_Array[Dial_Motor].targetAngle, M2006_Array[Dial_Motor].totalAngle);

    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);

    /********************检测堵转*****************/
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
 * @brief  拨弹处理函数2
 * @param  void
 * @retval void
 * @attention
 */
void Dial_Processing_2(void)//下午的更新：
{
  uint32_t current_time = HAL_GetTick(); // 获取当前时间
  static int32_t last_normal_angle = 0;   // 上一次正常拨弹时的总角度值
  static int32_t total_normal_angle = 0;  // 累计的正常拨弹角度（绝对值）
  static uint32_t last_cooling_time = 0;  // 上一次记录冷却的时间
  static uint32_t cooling_ticks = 0;      // 累计的冷却tick数

  if (Dial_Data.Shoot_Mode == Continuous_Shoot && Dial_Data.Dial_Switch == Dial_On)
  {
    
    /*************************正常拨弹执行/冷却时间计时/正常拨弹角度计数************************/
    // 初始化正常拨弹角度记录（首次进入或重置后）
    if (last_normal_angle == 0) 
    {
      last_normal_angle = M2006_Array[Dial_Motor].totalAngle;
    }

    // 计算冷却时间（仅当current_heat≥0时累计）
    uint32_t delta_time = current_time - last_cooling_time;
    last_cooling_time = current_time;
    if (Heat_Data.current_heat >= 0) 
    {
      cooling_ticks += delta_time;
    }

    if (!Heat_Data.overheat && !is_reversing)//没有过热,没有反转时正常拨弹
    {
      M2006_Array[Dial_Motor].targetSpeed = -2500; // Dial_Data.Speed_Dial;
      M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);

      // 仅在正常拨弹（非反转）时更新累计角度
      int32_t current_angle = M2006_Array[Dial_Motor].totalAngle;
      // 计算角度变化量（正常拨弹时角度递减，差值为正）
      int32_t delta_angle = last_normal_angle - current_angle;
      total_normal_angle += delta_angle;
      last_normal_angle = current_angle;
    }
    else//过热时停止拨弹
    {
      M2006_Array[Dial_Motor].targetSpeed = 0; // Dial_Data.Speed_Dial;
      M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
    }

    /********************拨弹盘卡弹检测*************************************************/
    // 定时检测角度变化(每CHECK_INTERVAL ms检测一次)
    if (current_time - last_check_time >= CHECK_INTERVAL)
    {
      // 计算角度变化量(取绝对值)
      int32_t angle_change = abs(M2006_Array[Dial_Motor].totalAngle - last_angle);

      // 如果角度变化小于阈值且没有处于过热停机状态，判断为卡弹
      if (angle_change < ANGLE_CHANGE_THRESHOLD && !Heat_Data.overheat && !is_reversing)
      {
        is_reversing = 1;
        reverse_start_time = current_time;
        // 立即开始反转
        M2006_Array[Dial_Motor].targetSpeed = 1000;
        M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
      }

      // 更新检测基准值
      last_check_time = current_time;
      last_angle = M2006_Array[Dial_Motor].totalAngle;
    }
    
  /**********************拨弹盘卡弹反转执行***************************************/
  // 如果正在反向
  if (is_reversing)
  {
    // 检测时间是否达到0.5秒
    if (current_time - reverse_start_time >= REVERSE_DURATION)
    {
      is_reversing = 0;// 取消反转
      last_normal_angle = M2006_Array[Dial_Motor].totalAngle;// 更新记录的角度，以便下一次正常拨弹时从新位置开始计算
    }
    else
    {
      // 不足0.5秒
      M2006_Array[Dial_Motor].targetSpeed = 1000; // Dial_Data.Speed_Dial;
      M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, M2006_Array[Dial_Motor].targetSpeed, M2006_Array[Dial_Motor].realSpeed);
      return; // 退出函数
    }
  }
    /**********************过热检测**************************************/
    //需要写热量累计以及热量冷却两方面的程序。
    //热量累计完全取决非反转时的角度变化
    //热量冷却值完全取决于时间的变化，要进行大于零的限幅。
    Heat_Data.current_heat = (total_normal_angle*10.0f/Angle_DialOneBullet_17mm) - (cooling_ticks*Heat_Data.cooling_rate/configTICK_RATE_HZ);
    if (Heat_Data.current_heat >= Heat_Data.max_heat) Heat_Data.overheat = 1;
  }
  else
  {
    M2006_Array[Dial_Motor].outCurrent = PID_Model4_Update(&M2006_DialI_Pid, &fuzzy_pid_bullet_v, 0, M2006_Array[Dial_Motor].realSpeed);
    Heat_Data.overheat = 0;  //过热标志刷新
    // 重置累计值
    // total_normal_angle = 0;
    // cooling_ticks = 0;
    last_normal_angle = M2006_Array[Dial_Motor].totalAngle;
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
 * @brief  PID重置
 * @param  void
 * @retval void
 * @attention
 */
void PID_Clear_Dial(void)
{

  Position_PIDInit(&M2006_DialV_Pid, 0.3f, 0.001f, 0.4, 0.5, 20000, 8000, 700); // 拨盘电机速度环
  Incremental_PIDInit(&M2006_DialI_Pid, 12.5f, 0.5f, 0, 10000, 1000);           // 拨盘电机电流环
}
