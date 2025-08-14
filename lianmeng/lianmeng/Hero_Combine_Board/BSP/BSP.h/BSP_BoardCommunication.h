/**
 * @file BSP_BoardCommunication.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-9-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BSP_BOARDCOMMUNICATION_H 
#define	BSP_BOARDCOMMUNICATION_H

#include "main.h"

#include "BSP_Fdcan.h"
#include "Extern_Handles.h"


// FDCAN���ĵı�ʶ�������ݳ���
#define FDCAN_ID_CHASSIS 0x10f // ����CAN���ĵ�������IDΪ0x10f
#define FDCAN_ID_GIMBAL  0x11f // ��̨����IDΪ0x11f
#define FDCAN_ID_SABER   0x22f // ����������IDΪ0x22f
#define FDCAN_ID_SABER_YAW 0x44f // ������(Yaw)����IDΪ0x22f
#define FDCAN_ID_SHOOT_HEAT 0x33f //����ϵͳǹ����������IDΪ0x33f
#define FDCAN_ID_CHASSIS_FUNCTION  0x55f  //���ܣ�����������

#define Board1_FunGroundInit   \
	{                          \
		&Board1_To_2,    	   \
		&Board1_getGimbalInfo, \
		&Board1_getSaberInfo,  \
		&Board1_getShootHeatInfo, \
	}

#define Board2_FunGroundInit   \
	{                          \
		&Board2_getChassisInfo,       \
		&Board2_getGimbalInfo,		  \
		&Board2_getChassisFunctionInfo,    \
		&Board2_To_1,                 \
	}


// ����CAN���ĵĽṹ��
typedef struct {
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_rotation_velocity;
	int16_t pitch_velocity;
	int16_t yaw_velocity;
	int16_t yaw_position;        		//����ʹ��ʱ��yaw��Ӧ���ڵľ���λ��
	uint8_t AutoAimFlag;         		//���鿪��
	uint8_t shoot_state;
	int16_t yaw_realAngle;       		//�°崫������yaw��Ƕ���Ϣ
	float   Speed_Bullet;        		//����ϵͳ�����ĵ���
	uint16_t shooter_42mm_heat_now;		//����ϵͳ�����ĵ�ǰǹ������
	uint16_t shooter_42mm_heat_limit;	//����ϵͳ������ǹ����������
	uint16_t chassis_power_buffer;  	//����ϵͳ�������ĵ��̹��ʻ��� ��λJ
	uint16_t chassis_power_limit;   	//����ϵͳ�������ĵ��̹�������
	float chassis_power;		    	//����ϵͳ�������ĵ����������   ��λw
	uint8_t tnndcolor;                  //��������ɫ��1Ϊ�죬2Ϊ��
	uint8_t fric_Flag;						//Ħ���ֿ���
	uint8_t Check_In_Flag;				//��¼����
	uint8_t Climb_Whole_Flag;           //�������أ����壩
	uint8_t Climb_Front_Flag;           //�������أ�ǰ�֣�
	uint8_t Climb_Back_Flag;            //�������أ����֣�������˳�������壬Ȼ����ǰ�֣�֮���պ��֣�
	uint8_t Deployment_Flag;      //����ģʽ����
	uint16_t Blood_Volume;         // ????
	uint8_t game_start;
} ControlMessge;

 typedef struct IMU_Angle_t
 {
	fp32 RoLL;
	fp32 Pitch;
	fp32 Yaw;
	 
	fp32 X_Vel;
	fp32 Y_Vel;
	fp32 Z_Vel;
	 
	fp32 X_Acc;
	fp32 Y_Acc;
	fp32 Z_Acc;
	 
	uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;
    uint32_t IMU_FPS;
	 
 }IMU_Angle_t;
extern IMU_Angle_t IMU_Angle;

extern ControlMessge ControlMes;

typedef struct
{
	void (*Board1_To_2)(void);
	void (*Board1_getGimbalInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board1_getSaberInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board1_getShootHeatInfo)(Fdcan_Export_Data_t RxMessage);
}Board1_FUN_t;

typedef struct
{
	void (*Board2_getChassisInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_getGimbalInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_getChassisFunctionInfo)(Fdcan_Export_Data_t RxMessage);
	void (*Board2_To_1)(void);
}Board2_FUN_t;


extern Board1_FUN_t Board1_FUN;
extern Board2_FUN_t Board2_FUN;

#endif
