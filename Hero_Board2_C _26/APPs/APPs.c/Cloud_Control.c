/**
 * @file Cloud_control.c
 * @author Cyx
 * @brief 云台控制模块，负责云台偏航轴电机的PID控制和卡尔曼滤波
 * @version 0.1
 * @date 2023-08-15
 * @copyright 版权信息
 */
#include "Cloud_Control.h"  // 包含云台控制头文件

/************电机PID控制器声明***********/
positionpid_t M6020s_YawIPID;          // 偏航轴电机内环PID（电流环）
positionpid_t M6020s_Yaw_SpeedPID;     // 偏航轴速度环PID（未使用）
positionpid_t M6020s_YawOPID;          // 偏航轴外环PID（位置环）
positionpid_t AutoAim_M6020s_YawIPID;  // 自动瞄准模式下的内环PID
positionpid_t AutoAim_M6020s_YawOPID;  // 自动瞄准模式下的外环PID
/************电机PID END***********/

/****************卡尔曼滤波器结构体创建*****************/
One_Kalman_t Cloud_YawMotorAngle_Error_Kalman;  // 偏航轴角度误差卡尔曼滤波
One_Kalman_t Cloud_YawCurrent_Kalman;           // 自动模式电流卡尔曼滤波
One_Kalman_t Cloud_YawCurrent_Kalman_manul;     // 手动模式电流卡尔曼滤波
/****************卡尔曼滤波结构体创建 End*****************/

/********变量声明********/
Cloud_t Cloud;            // 云台控制全局结构体
float Control_Self_Yaw;   // 自瞄控制偏航角（未使用）
float shit;               // 未使用的临时变量（建议删除）
/********数据声明********/
float Linear = 2.75f;             // 陀螺仪滞后补偿系数
float Setup_Angleoffset = -3000;  // 云台安装角度偏移量
uint8_t kk = 8;                   // 外环PID计算频率控制参数
/********全局变量声明********/
extern M6020s_t* M6020_Array[1];  // 电机数组指针（索引0对应ID1）
extern Saber_Angle_t Saber_Angle; // 外部导入的陀螺仪角度数据

/********函数声明********/
void Cloud_Init(void);            // 云台初始化
void Cloud_Yaw_Angle_Set(void);   // 偏航轴角度控制
void Cloud_Sport_Out(void);       // 云台控制输出
void Cloud_Self_Yaw(void);        // 自瞄功能（未实现）
void PID_Clear_Yaw(void);         // PID数据清除

/***************输出接口定义***************/
Cloud_FUN_t Cloud_FUN = Cloud_FUNGroundInit;  // 云台功能函数结构体初始化
#undef Cloud_FUNGroundInit  // 取消基础初始化宏定义

/**
 * @brief  云台初始化，配置参数并归位云台
 * @param  None
 * @retval None
 */
void Cloud_Init(void)
{
    // 初始化目标角度：当前电机角度 + 陀螺仪偏航角转换值
    // 陀螺仪角度(度)转换为编码器值(0-8191范围)
    Cloud.Target_Yaw = M6020s_Yaw.realAngle + Saber_Angle.Yaw / 360.0f * 8192.0f; 

    // 初始化卡尔曼滤波器
    One_Kalman_Create(&Cloud_YawMotorAngle_Error_Kalman, 1, 10);  // Q=1, R=10
    One_Kalman_Create(&Cloud_YAWODKalman, 1, 10);                // 未声明的滤波器（可能是笔误）
    One_Kalman_Create(&Cloud_YawCurrent_Kalman, 1, 6);           // 自动模式滤波器
    One_Kalman_Create(&Cloud_YawCurrent_Kalman_manul, 1, 6);     // 手动模式滤波器
}

/**
  * @brief  清除偏航轴PID控制器数据
  * @param  void
  * @retval void
  */
void PID_Clear_Yaw(void)
{
    Clear_PositionPIDData(&M6020s_YawIPID);  // 清除内环PID数据
    Clear_PositionPIDData(&M6020s_YawOPID);  // 清除外环PID数据
}

/**
  * @brief  偏航轴角度控制（核心函数）
  * @param  void
  * @retval void
  */
void Cloud_Yaw_Angle_Set(void)
{
    /*********************** 云台角度初始化更新 ***********************/
    // 当电机数据更新帧数有效时（≤30帧），更新目标角度
    if(M6020s_Yaw.InfoUpdateFrame <= 30) {
        Cloud.Target_Yaw = Saber_Angle.Yaw /360.0f * 8192.0f + M6020s_Yaw.realAngle;
    }

    // 目标角度归一化到0-8191范围（对应编码器一圈）
    if (Cloud.Target_Yaw > 8192) Cloud.Target_Yaw -= 8192;
    else if (Cloud.Target_Yaw < -8192) Cloud.Target_Yaw += 8192;

    /*********************** 坐标系转换 ***********************/
    // 将陀螺仪偏航角转换为编码器值范围
    float Angle_Yaw_Chassis = Saber_Angle.Yaw / 360.0f * 8192.0f;
    // 计算世界坐标系下的云台角度（底盘角度 + 电机角度）
    float Angle_Yaw_Cloud = M6020s_Yaw.realAngle + Angle_Yaw_Chassis;
    
    // 角度归一化处理（-4096~4096范围）
    if (Angle_Yaw_Cloud > 4096) Angle_Yaw_Cloud -= 8192;
    else if (Angle_Yaw_Cloud < -4096) Angle_Yaw_Cloud += 8192;
    
    ControlMes.yaw_realAngle = Angle_Yaw_Cloud;  // 更新控制信息中的实时角度

    /*********************** 角度偏差计算 ***********************/
    // 计算当前角度与目标角度的偏差，并补偿陀螺仪传输延迟
    float Delta_Yaw = Angle_Yaw_Cloud - Cloud.Target_Yaw + Linear * Saber_Angle.Z_Vel;
    
    // 偏差角度归一化处理
    if (Delta_Yaw <= -4096) Delta_Yaw += 8192;
    else if (Delta_Yaw >= 4096) Delta_Yaw -= 8192;

    /*********************** 控制模式选择 ***********************/
    static uint8_t time = 5;  // 外环PID计算计时器
    
    // 手动控制模式
    if(ControlMes.AutoAimFlag == 0) {
        // 死区处理：小角度偏差视为零
        if(Delta_Yaw < 5 && Delta_Yaw > -5) Delta_Yaw = 0;
        
        // 卡尔曼滤波平滑角度偏差
        Delta_Yaw = One_Kalman_Filter(&Cloud_YawMotorAngle_Error_Kalman, Delta_Yaw);
        
        // 外环PID计算（降频执行）
        if(time >= kk) {
            M6020s_Yaw.targetSpeed = Position_PID(&M6020s_YawOPID, 0, Delta_Yaw);
            time = 0;  // 重置计时器
        }
        time++;  // 计时器递增
        
        // 内环PID计算（带模糊PID）
        M6020s_Yaw.outCurrent = Position_PID_Yaw(&M6020s_YawIPID, &FuzzyPID_Yaw, 
                                                M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
        // 电流输出滤波
        M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman_manul, M6020s_Yaw.outCurrent);
    } 
    // 自动瞄准模式
    else if(ControlMes.AutoAimFlag == 1) {
        // 死区处理
        if(Delta_Yaw < 10 && Delta_Yaw > -10) Delta_Yaw = 0;
        
        // 使用自动瞄准专用PID计算目标速度
        M6020s_Yaw.targetSpeed = Position_PID(&AutoAim_M6020s_YawOPID, 0, Delta_Yaw);
        
        // 内环PID计算（带自动瞄准模糊PID）
        M6020s_Yaw.outCurrent = Position_PID_Yaw(&AutoAim_M6020s_YawIPID, &FuzzyPID_AimYaw,
                                                M6020s_Yaw.targetSpeed, M6020s_Yaw.realSpeed);
        // 电流输出滤波
        M6020s_Yaw.outCurrent = One_Kalman_Filter(&Cloud_YawCurrent_Kalman, M6020s_Yaw.outCurrent);
    }
}

/**
  * @brief  云台控制输出函数
  * @param  void
  * @retval void
  */
void Cloud_Sport_Out(void)
{
    /********** 状态检查 **********/
    // 录制模式直接返回
    if(ControlMes.modelFlag == model_Record) {
        M6020s_Yaw.InfoUpdateFrame = 0;
        return;
    } 
    // 电机数据更新时执行角度控制
    else if(M6020s_Yaw.InfoUpdateFlag == 1) {
        Cloud_FUN.Cloud_Yaw_Angle_Set();  // 调用角度控制函数
    } 
    // 无更新时直接返回
    else {
        return;
    }

    uint8_t data[8] = {0};  // CAN发送数据缓冲区

    /********** 云台角度反馈 **********/
    // 计算带偏移量的云台角度
    float Angle_Cloud = M6020s_Yaw.realAngle + Setup_Angleoffset;
    // 角度归一化（-4096~4096）
    if(Angle_Cloud > 4096) Angle_Cloud -= 8192;
    else if (Angle_Cloud < -4096) Angle_Cloud += 8192;
    // 发送角度给其他模块（转换为度）
    steer_getangle(-1 * Angle_Cloud / 8192.0f * 360);

    /********** 电机控制输出 **********/
    // 设置四个电机的电流值
    M6020_Fun.M6020_setVoltage(M6020s_Yaw.outCurrent, 
                              M6020s_Yaw.outCurrent,
                              M6020s_Yaw.outCurrent, 
                              M6020s_Yaw.outCurrent, 
                              data);
    // 通过CAN发送电流指令
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M6020_SENDID, data);
}