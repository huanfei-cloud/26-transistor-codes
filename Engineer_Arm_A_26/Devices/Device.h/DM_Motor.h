/**
 * @file DM_Motor.h
 * @author ZS
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c)
 *
 */
#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "can.h"
#include "main.h"
#include "typedef.h"
#include "Task_CanReceive.h"
#include "pid.h"

//J3(小臂电机)
//J4(大Roll电机)
//J5(小Roll电机)
//J6(Pitch电机)

//CAN2接收电机数据的初始ID和末ID
#define RECEIVE_START_ID 0x04
#define RECEIVE_END_ID 0x07

//J3电机常值数据
#define J4340_READID_J3 0x04
#define J4340_SENDID_J3 0x04

#define J4340_MaxV 20.0f        //发送给电机的最大转速,单位rpm
#define J4340_MaxT 10.0f        //发送给电机的最大扭矩，单位nm
#define J4340_MaxP 12.5f
#define J4340_ReductionRatio 40 //电机减速比
//J4电机常值数据
#define J4310_READID_J4 0x05
#define J4310_SENDID_J4 0x05 
//J5电机常值数据
#define J4310_READID_J5 0x06
#define J4310_SENDID_J5 0x06
//J6电机常值数据
#define J4310_READID_J6 0x07
#define J4310_SENDID_J6 0x07

#define J4310_MaxV 20.0f       //发送给电机的最大转速,单位rpm
#define J4310_MaxT 10.0f         //发送给电机的最大扭矩，单位NM
#define J4310_MaxP 12.5f
#define J4310_ReductionRatio 10 //电机减速比

//初始时各个电机的位置
#define J4340_FIRSTANGLE_J3 3800 /* 电机初始位置 */
#define J4310_FIRSTANGLE_J4 3800 /* 电机初始位置 */
#define J4310_FIRSTANGLE_J5 3800 /* 电机初始位置 */
#define J4310_FIRSTANGLE_J6 3800 /* 电机初始位置 */

#define Pi 3.1415926f


#define DM_FunGroundInit        \
    {                              \
		  &DM_setParameter,		   \
			&DM_Enable,      \
			&DM_MIT_Init,     \
			&DM_Save_Pos_Zero,    \
			&DM_getInfo,		   \
			&DM_setTargetAngle, \
			&DM_Reset,          \
			&Check_DM,		   \
    }

typedef struct
{
		int16_t  state; 	   //读回来的电机状态
    float realAngle;     //算出来的机械角度（单位：度）
    float realSpeed;     //算出来的速度（单位：rpm）
    uint8_t temperatureMOS;    //读回来的电机MOS温度
		uint8_t temperatureRotor;  //读回来的电机线圈温度
	  float  torqueInit;         //读回来的电机扭矩
	  float  torque;             //算出来的电机扭矩
	  float  angleInit;   //读回来的机械角度
    float  speedInit;   //读回来的速度
		
    uint16_t lastAngle;  //上次的角度
	
    int32_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
	
	  float outPosition;   //输出位置
	  float outSpeed;      //输出速度
	  float outTorque;     //输出扭矩
	
    int16_t turnCount;   //转过的圈数
    float totalAngle;    //累积总共角度

    int8_t outKp;        //位置比例系数
    int8_t outKd;        //位置微分系数

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
		
	  struct Struct_PID_Manage_Object l_pid_object;  //位置环PID
		
} DM_Motors_t;

typedef enum
{
    //需要注意与报文接受函数处对应。即
    J4340_J3 = 0,
	  J4310_J4,
	  J4310_J5,
	  J4310_J6,
	  totalnum,
} DMMotorName_e;


typedef struct
{
  void (*DM_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5,float p_max,float v_max,float t_max, uint8_t *data);
  void (*DM_Enable)(uint32_t id,CAN_HandleTypeDef *hcan);
	void (*DM_MIT_Init)(DM_Motors_t *DMmotor,float uq1,float uq2, float uq3, float uq4, float uq5);
  void (*DM_Save_Pos_Zero)(uint32_t id,CAN_HandleTypeDef *hcan);
  void (*DM_getInfo)(Can_Export_Data_t RxMessage,float p_max,float v_max,float t_max);
  void (*DM_setTargetAngle)(DM_Motors_t *DMmotor, int32_t angle);
  void (*DM_Reset)(DM_Motors_t *DMmotor);
  void (*Check_DM)(void);
} DM_Motors_Fun_t;

/********全局变量声明********/
extern DM_Motors_t J4340s_J3;
extern DM_Motors_t J4310s_J4;
extern DM_Motors_t J4310s_J5;
extern DM_Motors_t J4310s_J6;
extern DM_Motors_Fun_t DM_Motors_Fun;

/********函数声明********/
void DM_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5,float p_max,float v_max,float t_max, uint8_t *data);
void DM_Enable(uint32_t id,CAN_HandleTypeDef *hcan);
void DM_MIT_Init(DM_Motors_t *DMmotor,float uq1,float uq2, float uq3, float uq4, float uq5);
void DM_Save_Pos_Zero(uint32_t id,CAN_HandleTypeDef *hcan);
void DM_getInfo(Can_Export_Data_t RxMessage,float p_max,float v_max,float t_max);
void DM_setTargetAngle(DM_Motors_t *DMmotor, int32_t angle);
void DM_Reset(DM_Motors_t *DMmotor);
void Check_DM(void);
void DMmotor_location_change(DM_Motors_t *motor, enum pid_control model, float target,float real);
#endif /* __J4310_MOTOR_H */
