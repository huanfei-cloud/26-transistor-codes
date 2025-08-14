#ifndef __Omni_Chassis_H
#define __Omni_Chassis_H
#include "main.h"
#include "M2006_Motor.h"

#define OMNI_LENGTH_A 110/2 //���̳���һ��
#define OMNI_LENGTH_B 110/2 //���̿��һ��

#define OMNI_WHEEL_PERIMETER 80//ȫ����ֱ������λmm
//#define CHASSIS_DECELE_RATIO 36//2006�綯��ת�ٱȣ��ٷ��ֲ�Ϊ36

#define Omni_DataGroundInit         \
				{                      \
						{0},           \
						{0},           \
						 0 ,           \
						{0},           \
				}
				
#define Omni_FunGroundInit                       \
				{                                   \
						&Omni_Chassis_Out,   \
						&Omni_GetAngle, 			\
						&RemoteControlChassis,      \
				}

typedef struct
{
    struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToCloud;            //��������̨��������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
		
	struct
    {
        float vx;
        float vy;
        float vw;
    } Speed_ToChassis;          //�����ڵ��̳�������ϵ�µ��ٶȱ�����Vx,Vy,Vw��
		
		fp32 Angle_ChassisToCloud;
		int16_t M2006_Setspeed[4];	 //�ĸ�2006����Ŀ��ת��
} Omni_Data_t;

typedef struct
{
	void (*Omni_Chassis_Out)();
	void (*Omni_GetAngle)(fp32 angle);
	void (*RemoteControlChassis)(int16_t *speed);
}Omni_Fun_t;

extern Omni_Fun_t Omni_Fun;

#endif
