#include "Omni_Chassis.h"
#include "M2006_Motor.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"

/***************�û����ݶ���***************/
//void Chassis_open_init(void);
//void CHASSIS_InitArgument(void);
void Omni_calc(void);
//void Omni_angle_calc(float* out_angle) ;
void Omni_Set_Motor_Speed(M2006s_t* Motor);
void Omni_Absolute_Cal(fp32 angle)	;
void Omni_Chassis_Out();
//float Find_Y_AnglePNY(void);
//float Find_min_Angle(int16_t angle1, fp32 angle2);
void RemoteControlChassis(int16_t *speed);
void Omni_GetAngle(fp32 angle);

Omni_Data_t Omni_Data = Omni_DataGroundInit;
#undef Omni_DataGroundInit

/***************����ӿڶ���***************/
Omni_Fun_t Omni_Fun = Omni_FunGroundInit;
#undef Omni_FunGroundInit

/**
  * @brief  ��ȡң����ָ��
  * @param  speedΪ������̵ľ���Ŀ���ٶ�
  * @retval void
  * @attention ͨ��ң�����£����ڣ������̵�Ŀ���ٶ�
  */
void RemoteControlChassis(int16_t *speed)
{
    Omni_Data.Speed_ToCloud.vx = speed[0];
    Omni_Data.Speed_ToCloud.vy = speed[1];
    Omni_Data.Speed_ToCloud.vw = speed[2];
}

/**
  * @brief  ����̨����ת��Ϊ��������
  * @param  angle ��̨����ڵ��̵ĽǶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention �ٶ������ٶ�������̨Ϊ����ϵ���˺����������ٶ�ת������������ϵ��
  */
void Omni_Absolute_Cal(fp32 angle)
{
    fp32 angle_hd = angle * PI / 180;

    Omni_Data.Speed_ToChassis.vw = Omni_Data.Speed_ToCloud.vw;
    Omni_Data.Speed_ToChassis.vx = Omni_Data.Speed_ToCloud.vx * cos(angle_hd) - \
                                      Omni_Data.Speed_ToCloud.vy * sin(angle_hd);
    Omni_Data.Speed_ToChassis.vy = Omni_Data.Speed_ToCloud.vx * sin(angle_hd) + \
                                      Omni_Data.Speed_ToCloud.vy * cos(angle_hd);

    //��֤�������������ͷ���ƶ���������ͷת��90��ʱx�����ٶȴ�1��0��
    //y�����ٶȴ�0��1����֤�Ӿ������������

    Omni_calc();
}

/**
  * @brief  ���������������ĵ��ٶȷֽ�Ϊ�ĸ������ֵ��ٶ�
  * @param  speedΪ��������ϵ�����ĵ��ٶ�
  * @param  out_speedΪ�ĸ��ֵ�����ٶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention �ٶ������ٶ�������̨Ϊ����ϵ���˺����������ٶ�ת������������ϵ��
  */
void Omni_calc()
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;
    //��Ҫ�����ٶ�ת��Ϊת��rpm
    wheel_rpm_ratio = 60.0f / (OMNI_WHEEL_PERIMETER * 3.14f) * M2006_ReductionRatio;

    /*x��y�����ٶ�,w����ת���ٶ�*/
    wheel_rpm[0] = ( Omni_Data.Speed_ToChassis.vx + Omni_Data.Speed_ToChassis.vy + \
                     Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //left
    wheel_rpm[1] = ( Omni_Data.Speed_ToChassis.vx - Omni_Data.Speed_ToChassis.vy + \
                     Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //forward
    wheel_rpm[2] = (-Omni_Data.Speed_ToChassis.vx - Omni_Data.Speed_ToChassis.vy + \
                    Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //right
    wheel_rpm[3] = (-Omni_Data.Speed_ToChassis.vx + Omni_Data.Speed_ToChassis.vy + \
                    Omni_Data.Speed_ToChassis.vw * (OMNI_LENGTH_A + OMNI_LENGTH_B)) * wheel_rpm_ratio; //back

    memcpy(Omni_Data.M2006_Setspeed, wheel_rpm, sizeof(wheel_rpm)); //copy the rpm to out_speed
}

/**
  * @brief  ����2006Ŀ���ٶ�
  * @param  Motor ����ṹ��
  * @retval
  * @attention
  */
void Omni_Set_Motor_Speed(M2006s_t *Motor )
{
    Motor[0].targetSpeed = Omni_Data.M2006_Setspeed[0];
    Motor[1].targetSpeed = Omni_Data.M2006_Setspeed[1];
    Motor[2].targetSpeed = Omni_Data.M2006_Setspeed[2];
    Motor[3].targetSpeed = Omni_Data.M2006_Setspeed[3];
}

/**
  * @brief  ���̵�����
  * @param  void
  * @retval void
  * @attention
  */
void Omni_Chassis_Out()
{
    uint8_t data[8] = {0};

    Omni_Absolute_Cal(Omni_Data.Angle_ChassisToCloud); //������������Ŀ���ٶ�
    Omni_Set_Motor_Speed(M2006_Array); 							 //���ø��������Ŀ���ٶ�

    //Chassis_Power_Limit();//��������

    /**************************����2006����ٶȻ�����*****************************/
//		 speed[i] = Wheel_PID[i]->Incremental_PID(Wheel_PID[i], M3508s[i].targetSpeed, M3508s[i].realSpeed);
    M2006_Array[0].outCurrent = Incremental_PID(&M2006_Array_Pid[0],
                                M2006_Array[0].targetSpeed,
                                M2006_Array[0].realSpeed);
    M2006_Array[1].outCurrent = Incremental_PID(&M2006_Array_Pid[1],
                                M2006_Array[1].targetSpeed,
                                M2006_Array[1].realSpeed);
    M2006_Array[2].outCurrent = Incremental_PID(&M2006_Array_Pid[2],
                                M2006_Array[2].targetSpeed,
                                M2006_Array[2].realSpeed);
    M2006_Array[3].outCurrent = Incremental_PID(&M2006_Array_Pid[3],
                                M2006_Array[3].targetSpeed,
                                M2006_Array[3].realSpeed);

    /***************************�������������͸����*******************************/
    M2006_FUN.M2006_setCurrent(M2006_Array[0].outCurrent, M2006_Array[1].outCurrent,
                               M2006_Array[2].outCurrent, M2006_Array[3].outCurrent,
                               data);
    Can_Fun.CAN_SendData(CAN_SendHandle, &hcan1, CAN_ID_STD, M2006_SENDID, data);
}

void Omni_GetAngle(fp32 angle)
{
    Omni_Data.Angle_ChassisToCloud = angle  ;
}


