
#include "Omni_Chassis.h"   // 全向底盘控制相关定义
#include "BSP_Can.h"        // CAN总线驱动
#include "Extern_Handles.h" // 外设句柄
#include "PowerControl.h"   // 电源控制

/* 用户数据定义 (已注释的函数声明可能是早期版本的功能)
//void Chassis_open_init(void);
//void CHASSIS_InitArgument(void);
//void Omni_angle_calc(float* out_angle) ;
//float Find_Y_AnglePNY(void);
//float Find_min_Angle(int16_t angle1, fp32 angle2);
*/

// 函数声明
void Omni_calc(void);                           // 运动学解算函数
void Omni_Set_Motor_Speed(M3508s_t* Motor);      // 设置电机速度
void Omni_Absolute_Cal(fp32 angle);              // 坐标系转换函数
void Omni_Chassis_Out(void);                     // 底盘输出控制
void RemoteControlChassis(int16_t *speed);        // 遥控指令处理
void Omni_GetAngle(fp32 angle);                 // 获取云台角度

// 全局底盘数据结构实例，初始化为默认值
Omni_Data_t Omni_Data = Omni_DataGroundInit;     
#undef Omni_DataGroundInit  // 取消宏定义，防止重复使用

// 底盘功能接口结构体实例
Omni_Fun_t Omni_Fun = Omni_FunGroundInit;        
#undef Omni_FunGroundInit   // 取消宏定义，防止重复使用

// 四个M3508电机的增量式PID控制器
incrementalpid_t M3508_Array_Pid[4];		

// 灵敏度系数，用于调节速度和旋转响应
float SEN_SPEED = 1.5;    // 速度灵敏度系数
float SEN_ROTATE = 0.5;   // 旋转灵敏度系数

/**
  * @brief  获取遥控器控制指令
  * @param  speed: 遥控器输入的速度数组 [vx, vy, vw]
  * @retval void
  * @attention 通过遥控器输入，设置底盘的目标速度
  */
void RemoteControlChassis(int16_t *speed)
{
    // 保存遥控器发送的速度指令到数据结构
    Omni_Data.Speed_ToCloud.vx = speed[0];  // X轴速度分量
    Omni_Data.Speed_ToCloud.vy = speed[1];  // Y轴速度分量
    Omni_Data.Speed_ToCloud.vw = speed[2];  // 旋转速度分量
}

/**
  * @brief  将云台坐标系速度转换为底盘坐标系速度
  * @param  angle: 云台相对于底盘的旋转角度 (度)
  * @retval void
  * @attention 速度输入是云台坐标系，此函数将其转换为底盘坐标系
  */
void Omni_Absolute_Cal(fp32 angle)
{
    // 将角度从度转换为弧度
    fp32 angle_hd = angle * PI / 180;

    // 旋转速度直接映射（应用灵敏度系数）
    Omni_Data.Speed_ToChassis.vw = Omni_Data.Speed_ToCloud.vw * SEN_ROTATE;
    
    // 使用旋转矩阵转换线性速度：
    // 新X速度 = 原X速度*cos(角度) - 原Y速度*sin(角度)
    Omni_Data.Speed_ToChassis.vx = Omni_Data.Speed_ToCloud.vx * cos(angle_hd) - \
                                      Omni_Data.Speed_ToCloud.vy * sin(angle_hd);
    
    // 新Y速度 = 原X速度*sin(角度) + 原Y速度*cos(角度)
    Omni_Data.Speed_ToChassis.vy = Omni_Data.Speed_ToCloud.vx * sin(angle_hd) + \
                                      Omni_Data.Speed_ToCloud.vy * cos(angle_hd);

    // 功能说明：当云台旋转90°时，原X速度变为新Y速度，原Y速度变为新X速度的负值
    // 确保底盘移动方向与操作者视角一致（当云台转向时，前进方向始终是操作者的前方）
    
    // 调用运动学解算函数，计算电机转速
    Omni_calc();
}

/**
  * @brief  运动学解算：将底盘中心速度分解为四个轮子转速
  * @retval void
  * @attention 速度输入是底盘坐标系，输出是四个电机的目标转速
  */
void Omni_calc()
{
    int16_t wheel_rpm[4];        // 存储计算得到的四个轮子的RPM值
    float wheel_rpm_ratio;       // 速度转RPM的转换系数
    
    // 计算速度到RPM的转换系数：
    // RPM = 速度(m/s) * (60s/min) / (轮子周长(m)) * 减速比
    wheel_rpm_ratio = 60.0f / (OMNI_WHEEL_PERIMETER * 3.14f) * M3508_ReductionRatio;

    /* 全向轮运动学方程 - 基于麦克纳姆轮几何布局 */
    
    // 左前侧轮计算：
    // 速度 = Vx + Vy + 旋转速度 × (轴距A + 轴距B)
    wheel_rpm[0] = ((Omni_Data.Speed_ToChassis.vx-Omni_Data.Speed_ToChassis.vw*Length_steer_y)*cos(lf_steer_angle/180*pi)+
                    (Omni_Data.Speed_ToChassis.vy+Omni_Data.Speed_ToChassis.vw*Length_steer_x)*sin(lf_steer_angle/180*pi)) * wheel_rpm_ratio; 
    
    // 左后侧轮计算：
    wheel_rpm[1] = ((Omni_Data.Speed_ToChassis.vx-Omni_Data.Speed_ToChassis.vw*Length_steer_y)*cos(lb_steer_angle/180*pi)+
                    (Omni_Data.Speed_ToChassis.vy+Omni_Data.Speed_ToChassis.vw*Length_steer_x)*sin(lb_steer_angle/180*pi)) * wheel_rpm_ratio; 
    // 右后侧轮计算：
    wheel_rpm[3] = ((Omni_Data.Speed_ToChassis.vx-Omni_Data.Speed_ToChassis.vw*Length_steer_y)*cos(rb_steer_angle/180*pi)+
                    (Omni_Data.Speed_ToChassis.vy+Omni_Data.Speed_ToChassis.vw*Length_steer_x)*sin(rb_steer_angle/180*pi)) * wheel_rpm_ratio; 
    // 右前侧轮计算：
    wheel_rpm[2] =((Omni_Data.Speed_ToChassis.vx-Omni_Data.Speed_ToChassis.vw*Length_steer_y)*cos(rf_steer_angle/180*pi)+
                    (Omni_Data.Speed_ToChassis.vy+Omni_Data.Speed_ToChassis.vw*Length_steer_x)*sin(rf_steer_angle/180*pi)) * wheel_rpm_ratio; 

    // 将计算结果复制到底盘数据结构的M2006_Setspeed数组
    memcpy(Omni_Data.M3508_Setspeed, wheel_rpm, sizeof(wheel_rpm)); 
}

/**
  * @brief  设置电机目标转速
  * @param  Motor: 电机结构体数组指针
  * @retval void
  * @attention 将计算的目标速度应用到电机对象，并考虑全局速度灵敏度系数
  */
void Omni_Set_Motor_Speed(M3508s_t *Motor )
{
    // 设置每个电机的目标转速（应用灵敏度系数）
    Motor[0].targetSpeed = Omni_Data.M3508_Setspeed[0]*SEN_SPEED;
    Motor[1].targetSpeed = Omni_Data.M3508_Setspeed[1]*SEN_SPEED;
    Motor[2].targetSpeed = Omni_Data.M3508_Setspeed[2]*SEN_SPEED;
    Motor[3].targetSpeed = Omni_Data.M3508_Setspeed[3]*SEN_SPEED;
}

/**
  * @brief  底盘控制输出函数
  * @retval void
  * @attention 这是主控制循环中调用的函数，完成整个控制流程
  */
void Omni_Chassis_Out()
{
    uint8_t data[8] = {0};  // CAN数据发送缓冲区

    // 1. 坐标系转换：云台坐标 -> 底盘坐标
    Omni_Absolute_Cal(Omni_Data.Angle_ChassisToCloud); 
    
    // 2. 设置电机目标速度
    Omni_Set_Motor_Speed(M3508_Array); 				   
    
    /************************** PID控制器计算 *****************************/
    // 使用增量式PID计算电机输出电流
    
    // 电机0控制计算
    M3508_Array[0].outCurrent = Incremental_PID(&M3508_Array_Pid[0],
                                M3508_Array[0].targetSpeed,      // 目标速度
                                M3508_Array[0].realSpeed);       // 实际速度
                                
    // 电机1控制计算
    M3508_Array[1].outCurrent = Incremental_PID(&M3508_Array_Pid[1],
                                M3508_Array[1].targetSpeed,
                                M3508_Array[1].realSpeed);
                                
    // 电机2控制计算
    M3508_Array[2].outCurrent = Incremental_PID(&M3508_Array_Pid[2],
                                M3508_Array[2].targetSpeed,
                                M3508_Array[2].realSpeed);
                                
    // 电机3控制计算
    M3508_Array[3].outCurrent = Incremental_PID(&M3508_Array_Pid[3],
                                M3508_Array[3].targetSpeed,
                                M3508_Array[3].realSpeed);
		
    /*************************** 发送CAN指令 *******************************/
    // 设置CAN数据帧内容
    M3508_FUN.M3508_setCurrent(
        M3508_Array[0].outCurrent,  // 电机0电流值
        M3508_Array[1].outCurrent,  // 电机1电流值
        M3508_Array[2].outCurrent,  // 电机2电流值
        M3508_Array[3].outCurrent,  // 电机3电流值
        data);                      // 输出缓冲区
    
    // 通过CAN1发送控制命令到底盘电机
    Can_Fun.CAN_SendData(
        CAN_SendHandle,         // CAN发送句柄
        &hcan1,                 // CAN外设实例
        CAN_ID_STD,             // 标准ID模式
        M3508_SENDID_Chassis,   // 底盘电机CAN ID
        data);                  // 包含电流值的数据缓冲区
}

/**
  * @brief  更新云台与底盘相对角度
  * @param  angle: 云台相对于底盘的角度（度）
  * @retval void
  * @attention 从云台控制模块获取最新的角度值
  */
void Omni_GetAngle(fp32 angle)
{
    Omni_Data.Angle_ChassisToCloud = angle;  // 保存角度值用于坐标变换
}