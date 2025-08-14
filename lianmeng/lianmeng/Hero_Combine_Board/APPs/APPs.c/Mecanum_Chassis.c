
/**
 * @file Mecanum_Chassis.c
 * @author Lxr
 * @brief O型麦轮底盘控制
 * @version 0.1
 * @date 2023-08-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
 #include "Mecanum_Chassis.h"
 #include "M3508_Motor.h"
 #include "BSP_Fdcan.h"
 #include "SBUS.h"
 #include "Extern_Handles.h"
 
 /***************用户数据定义***************/
void Mecanum_calc(void);
void Mecanum_GetAngle(fp32 angle);
void Mecanum_Absolute_Cal(fp32 angle);
void Mecanum_Set_Motor_Speed(M3508s_t *Motor );
void Mecanum_Chassis_out(void);
void Mecanum_Chassis_Follow_Gimbal(void);

 /***************输出接口定义***************/
Mecanum_Fun_t Mecanum_Fun = Mecanum_FunGroundInit;
#undef Mecanum_FunGroundInit
Mecanum_Data_t Mecanum_Data = Mecanum_DataGroundInit;
#undef Mecanum_DataGroundInit

incrementalpid_t M3508_Chassis_Pid[4];
 /***************底盘跟随PID***************/
positionpid_t Chassis_Follow_Pid;
/**
  * @brief  将底盘坐标下质心的速度分解为四个驱动轮的速度
  * @param  speed为底盘坐标系下质心的速度
  * @param  out_speed为四个轮的输出速度
  * @retval 偏差角，角度制
  * @attention 假定给定速度是以云台为坐标系，此函数将给定速度转化到底盘坐标系下
  */
void Mecanum_calc(void)
 {
	  int16_t wheel_rpm[4];
    float wheel_rpm_ratio;
	//需要将线速度转化为转速
    wheel_rpm_ratio = 60.0f / (MECANUM_WHEEL_PERIMETER * 3.14159f) * M3508_ReductionRatio;
	
	/*x,y方向速度，w底盘转动速度*/
	wheel_rpm[0] = -(Mecanum_Data.Speed_ToChassis.vx + Mecanum_Data.Speed_ToChassis.vy +\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//右前
	wheel_rpm[1] = (Mecanum_Data.Speed_ToChassis.vx - Mecanum_Data.Speed_ToChassis.vy -\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//左前
	wheel_rpm[2] = (Mecanum_Data.Speed_ToChassis.vx + Mecanum_Data.Speed_ToChassis.vy -\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//左后
	wheel_rpm[3] = -(Mecanum_Data.Speed_ToChassis.vx - Mecanum_Data.Speed_ToChassis.vy +\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//右后
    memcpy(Mecanum_Data.M3508_Setspeed,wheel_rpm, sizeof(wheel_rpm));
 }
 
 /**
  * @brief  将云台坐标下的速度转换为底盘坐标下的速度
  * @param  angle 云台相对于底盘的角度
  * @retval 偏差角，角度制
  * @attention 假定给定速度是以云台为坐标系，此函数将给定速度转化到底盘坐标系下
  */
void Mecanum_Absolute_Cal(fp32 angle)
{
    fp32 angle_hd = angle * PI / 180;

    Mecanum_Data.Speed_ToChassis.vw = Mecanum_Data.Speed_ToCloud.vw;
    Mecanum_Data.Speed_ToChassis.vx = Mecanum_Data.Speed_ToCloud.vx * cos(angle_hd) - \
                                      Mecanum_Data.Speed_ToCloud.vy * sin(angle_hd);
    Mecanum_Data.Speed_ToChassis.vy = Mecanum_Data.Speed_ToCloud.vx * sin(angle_hd) + \
                                      Mecanum_Data.Speed_ToCloud.vy * cos(angle_hd);

    //保证底盘是相对摄像头做移动，当摄像头转过90度时x方向速度从1变0，
    //y方向速度从0变1，保证视觉上是相对右移

    Mecanum_calc();
}

/**
  * @brief  设置3508目标速度
  * @param  Motor 电机结构体
  * @retval
  * @attention
  */
void Mecanum_Set_Motor_Speed(M3508s_t *Motor )
{
    Motor[0].targetSpeed = Mecanum_Data.M3508_Setspeed[0];
    Motor[1].targetSpeed = Mecanum_Data.M3508_Setspeed[1];
    Motor[2].targetSpeed = Mecanum_Data.M3508_Setspeed[2];
    Motor[3].targetSpeed = Mecanum_Data.M3508_Setspeed[3];
}
/**
  * @brief 获取底盘与云台间的角度
* 单位:°
  */
void Mecanum_GetAngle(fp32 angle)
{
    Mecanum_Data.Angle_ChassisToCloud = angle ;
}


/**
  * @brief  底盘电机输出
  * @param  void
  * @retval void
  * @attention
  */
void Mecanum_Chassis_out()
{

	uint8_t data[8] = {0};
	Mecanum_Absolute_Cal(Mecanum_Data.Angle_ChassisToCloud); 	//计算各个电机的目标速度

	Mecanum_Set_Motor_Speed(M3508_Array); 						 //设置各个电机的目标速度
	
	 /**************************底盘3508电机速度环计算*****************************/
	M3508_Array[0].outCurrent = Incremental_PID(&M3508_Chassis_Pid[0],
								M3508_Array[0].targetSpeed,
								M3508_Array[0].realSpeed);
	M3508_Array[1].outCurrent = Incremental_PID(&M3508_Chassis_Pid[1],
								M3508_Array[1].targetSpeed,
								M3508_Array[1].realSpeed);
	M3508_Array[2].outCurrent = Incremental_PID(&M3508_Chassis_Pid[2],
								M3508_Array[2].targetSpeed,
								M3508_Array[2].realSpeed);
	M3508_Array[3].outCurrent = Incremental_PID(&M3508_Chassis_Pid[3],
								M3508_Array[3].targetSpeed,
								M3508_Array[3].realSpeed);
	
   /***************************将电流参数发送给电机*******************************/
	
	M3508_FUN.M3508_setCurrent(M3508_Array[0].outCurrent,M3508_Array[1].outCurrent,
							   M3508_Array[2].outCurrent,M3508_Array[3].outCurrent,
							   data);
	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle,&hfdcan1,FDCAN_STANDARD_ID,M3508_SENDID_Chassis,data);
}


/**
  * @brief  底盘一键跟随云台
  * @param  void
  * @retval void
  * @attention 驱动底盘使得底盘的正方向朝向云台方向
  */
void Mecanum_Chassis_Follow_Gimbal(void)
{
//	-298~62
//	-180~180
	fp32 angle_err = Mecanum_Data.Angle_ChassisToCloud;//单位是度°
	if(angle_err < -180)
	angle_err+=360;	
	Mecanum_Data.Speed_ToCloud.vw = -1*Position_PID(&Chassis_Follow_Pid,0,angle_err);
	
}