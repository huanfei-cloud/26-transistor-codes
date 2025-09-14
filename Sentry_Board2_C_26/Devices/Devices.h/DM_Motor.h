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
#include "PID.h"

//Yaw轴电机常值数据
#define J6006_READID_YAW 0x05
#define J6006_SENDID_Yaw 0x05
#define J6006_MaxV 20.0f         //发送给电机的最大转速,单位rpm
#define J6006_MaxT 10.0f           //发送给电机的最大扭矩，单位nm
#define J6006_MaxP 10.0f
#define J6006_ReductionRatio 6 //电机减速比
#define J4310_READID_PITCH 0x01
#define J4310_SENDID_Pitch 0x001 //控制Pitch轴电机
#define J4310_MaxV 200 //发送给电机的最大转速,单位rpm
#define J4310_MaxT 7 //发送给电机的最大扭矩，单位NM
#define J4310_ReductionRatio 10 //电机减速比

#define J4310_FIRSTANGLE 3800 /* 电机初始位置 */

#define J4310_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define Pi 3.1415926f

#define J4310_getRoundAngle(rAngle) rAngle / J4310_mAngleRatio //机械角度与真实角度的比率

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
    float realAngle;  //算出来的机械角度（单位：度）
    float realSpeed;   //算出来的速度（单位：rpm）
    uint8_t temperatureMOS;    //读回来的电机MOS温度
		uint8_t temperatureRotor;  //读回来的电机线圈温度
	  float  torqueInit;       //读回来的电机扭矩
	  float  torque;           //算出来的电机扭矩
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

    int8_t outKp;          //位置比例系数
    int8_t outKd;          //位置微分系数

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
		
//		positionpid_t v_pid_object;  //速度环PID
//	  positionpid_t l_pid_object;  //位置环PID
} DM_Motors_t;

typedef enum
{
    //需要注意与报文接受函数处对应。即
    //J4310_PITCH_Right = 0,
    J4310_PITCH = 0,
	  J6006_YAW,
	  totalnum,
} DMMotorName_e;


typedef struct
{
  void (*DM_setParameter)(float uq1, float uq2, float uq3, float uq4, float uq5,float p_max,float v_max,float t_max, uint8_t *data);
  void (*DM_Enable)(uint32_t id,CAN_HandleTypeDef *hcan);
	void (*DM_MIT_Init)(DM_Motors_t *DMmotor,float uq1,float uq2, float uq3, float uq4, float uq5);
  void (*DM_Save_Pos_Zero)(uint32_t id,CAN_HandleTypeDef *hcan);
  void (*DM_getInfo)(Can_Export_Data_t RxMessage,int model,float p_max,float v_max,float t_max);
  void (*DM_setTargetAngle)(DM_Motors_t *DMmotor, int32_t angle);
  void (*DM_Reset)(DM_Motors_t *DMmotor);
  void (*Check_DM)(void);
} DM_Motors_Fun_t;

/********全局变量声明********/
extern DM_Motors_t J4310s_Pitch;
extern DM_Motors_t J6006s_Yaw;
extern DM_Motors_Fun_t DM_Motors_Fun;

/********函数声明********/
void DM_setParameter(float uq1, float uq2, float uq3, float uq4, float uq5,float p_max,float v_max,float t_max, uint8_t *data);
void DM_Enable(uint32_t id,CAN_HandleTypeDef *hcan);
void DM_MIT_Init(DM_Motors_t *DMmotor,float uq1,float uq2, float uq3, float uq4, float uq5);
void DM_Save_Pos_Zero(uint32_t id,CAN_HandleTypeDef *hcan);
void DM_getInfo(Can_Export_Data_t RxMessage,int model,float p_max,float v_max,float t_max);
void DM_setTargetAngle(DM_Motors_t *DMmotor, int32_t angle);
void DM_Reset(DM_Motors_t *DMmotor);
void Check_DM(void);
#endif /* __J4310_MOTOR_H */
