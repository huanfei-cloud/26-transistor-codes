#include "Omni_Chassis.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"
#include "PowerControl.h"

/***************用户数据定义***************/
//void Chassis_open_init(void);
//void CHASSIS_InitArgument(void);
void Omni_calc(void);
//void Omni_angle_calc(float* out_angle) ;
void Omni_Set_Motor_Speed(M3508s_t* Motor);
void Omni_Absolute_Cal(fp32 angle)	;
void Omni_Chassis_Out(void);
//float Find_Y_AnglePNY(void);
//float Find_min_Angle(int16_t angle1, fp32 angle2);
void RemoteControlChassis(int16_t *speed);
void Omni_GetAngle(fp32 angle);

Omni_Data_t Omni_Data = Omni_DataGroundInit;
#undef Omni_DataGroundInit

/***************输出接口定义***************/
Omni_Fun_t Omni_Fun = Omni_FunGroundInit;
#undef Omni_FunGroundInit

incrementalpid_t M3508_Array_Pid[4];		//四个2006轮子PID结构体	
float SEN_SPEED = 1.5;
float SEN_ROTATE = 0.5;
/**
  * @brief  获取遥控器指令
  * @param  speed为传入底盘的绝对目标速度
  * @retval void
  * @attention 通过遥控器下（串口）发底盘的目标速度
  */
void RemoteControlChassis(int16_t *speed)
{
    Omni_Data.Speed_ToCloud.vx = speed[0];
    Omni_Data.Speed_ToCloud.vy = speed[1];
    Omni_Data.Speed_ToCloud.vw = speed[2];
}

/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  angle 云台相对于底盘的角度
  * @retval 偏差角，角度制
  * @attention 假定给定速度是以云台为坐标系，此函数将给定速度转化到底盘坐标系下
  */
void Omni_Absolute_Cal(fp32 angle)
{
    fp32 angle_hd = angle * PI / 180;

    Omni_Data.Speed_ToChassis.vw = Omni_Data.Speed_ToCloud.vw * SEN_ROTATE;
    Omni_Data.Speed_ToChassis.vx = Omni_Data.Speed_ToCloud.vx * cos(angle_hd) - \
                                      Omni_Data.Speed_ToCloud.vy * sin(angle_hd);
    Omni_Data.Speed_ToChassis.vy = Omni_Data.Speed_ToCloud.vx * sin(angle_hd) + \
                                      Omni_Data.Speed_ToCloud.vy * cos(angle_hd);

    //保证底盘是相对摄像头做移动，当摄像头转过90度时x方向速度从1变0，
    //y方向速度从0变1，保证视觉上是相对右移

    Omni_calc();
}

/**
  * @brief  将底盘坐标下质心的速度分解为四个驱动轮的速度
  * @param  speed为底盘坐标系下质心的速度
  * @param  out_speed为四个轮的输出速度
  * @retval 偏差角，角度制
  * @attention 假定给定速度是以云台为坐标系，此函数将给定速度转化到底盘坐标系下
  */
void Omni_calc()
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;
    //需要将线速度转化为转速rpm
    wheel_rpm_ratio = 60.0f / (OMNI_WHEEL_PERIMETER * 3.14f) * M3508_ReductionRatio;

    /*x，y方向速度,w底盘转动速度*/
    wheel_rpm[0] = ( Omni_Data.Speed_ToChassis.vx + Omni_Data.Speed_ToChassis.vy + \
                     Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //left
    wheel_rpm[1] = ( Omni_Data.Speed_ToChassis.vx - Omni_Data.Speed_ToChassis.vy + \
                     Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //forward
    wheel_rpm[3] = (-Omni_Data.Speed_ToChassis.vx - Omni_Data.Speed_ToChassis.vy + \
                    Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //right
    wheel_rpm[2] = (-Omni_Data.Speed_ToChassis.vx + Omni_Data.Speed_ToChassis.vy + \
                    Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //back

    memcpy(Omni_Data.M2006_Setspeed, wheel_rpm, sizeof(wheel_rpm)); //copy the rpm to out_speed
}

/**
  * @brief  设置2006目标速度
  * @param  Motor 电机结构体
  * @retval
  * @attention
  */
void Omni_Set_Motor_Speed(M3508s_t *Motor )
{
    Motor[0].targetSpeed = Omni_Data.M2006_Setspeed[0]*SEN_SPEED;
    Motor[1].targetSpeed = Omni_Data.M2006_Setspeed[1]*SEN_SPEED;
    Motor[2].targetSpeed = Omni_Data.M2006_Setspeed[2]*SEN_SPEED;
    Motor[3].targetSpeed = Omni_Data.M2006_Setspeed[3]*SEN_SPEED;
}

/**
  * @brief  底盘电机输出
  * @param  void
  * @retval void
  * @attention
  */
void Omni_Chassis_Out()
{
    uint8_t data[8] = {0};

    Omni_Absolute_Cal(Omni_Data.Angle_ChassisToCloud); //计算各个电机的目标速度
    Omni_Set_Motor_Speed(M3508_Array); 				   //设置各个电机的目标速度

    /**************************底盘2006电机速度环计算*****************************/
//		 speed[i] = Wheel_PID[i]->Incremental_PID(Wheel_PID[i], M3508s[i].targetSpeed, M3508s[i].realSpeed);
    M3508_Array[0].outCurrent = Incremental_PID(&M3508_Array_Pid[0],
                                M3508_Array[0].targetSpeed,
                                M3508_Array[0].realSpeed);
    M3508_Array[1].outCurrent = Incremental_PID(&M3508_Array_Pid[1],
                                M3508_Array[1].targetSpeed,
                                M3508_Array[1].realSpeed);
    M3508_Array[2].outCurrent = Incremental_PID(&M3508_Array_Pid[2],
                                M3508_Array[2].targetSpeed,
                                M3508_Array[2].realSpeed);
    M3508_Array[3].outCurrent = Incremental_PID(&M3508_Array_Pid[3],
                                M3508_Array[3].targetSpeed,
                                M3508_Array[3].realSpeed);
		
    /***************************将电流参数发送给电机*******************************/
    M3508_FUN.M3508_setCurrent(M3508_Array[0].outCurrent, M3508_Array[1].outCurrent,
                               M3508_Array[2].outCurrent, M3508_Array[3].outCurrent,
                               data);
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M3508_SENDID_Chassis, data);
}

void Omni_GetAngle(fp32 angle)
{
    Omni_Data.Angle_ChassisToCloud = angle  ;
}


