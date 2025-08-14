
/**
 * @file Mecanum_Chassis.c
 * @author Lxr
 * @brief O�����ֵ��̿���
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
 
 /***************�û����ݶ���***************/
void Mecanum_calc(void);
void Mecanum_GetAngle(fp32 angle);
void Mecanum_Absolute_Cal(fp32 angle);
void Mecanum_Set_Motor_Speed(M3508s_t *Motor );
void Mecanum_Chassis_out(void);
void Mecanum_Chassis_Follow_Gimbal(void);

 /***************����ӿڶ���***************/
Mecanum_Fun_t Mecanum_Fun = Mecanum_FunGroundInit;
#undef Mecanum_FunGroundInit
Mecanum_Data_t Mecanum_Data = Mecanum_DataGroundInit;
#undef Mecanum_DataGroundInit

incrementalpid_t M3508_Chassis_Pid[4];
 /***************���̸���PID***************/
positionpid_t Chassis_Follow_Pid;
/**
  * @brief  ���������������ĵ��ٶȷֽ�Ϊ�ĸ������ֵ��ٶ�
  * @param  speedΪ��������ϵ�����ĵ��ٶ�
  * @param  out_speedΪ�ĸ��ֵ�����ٶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention �ٶ������ٶ�������̨Ϊ����ϵ���˺����������ٶ�ת������������ϵ��
  */
void Mecanum_calc(void)
 {
	  int16_t wheel_rpm[4];
    float wheel_rpm_ratio;
	//��Ҫ�����ٶ�ת��Ϊת��
    wheel_rpm_ratio = 60.0f / (MECANUM_WHEEL_PERIMETER * 3.14159f) * M3508_ReductionRatio;
	
	/*x,y�����ٶȣ�w����ת���ٶ�*/
	wheel_rpm[0] = -(Mecanum_Data.Speed_ToChassis.vx + Mecanum_Data.Speed_ToChassis.vy +\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//��ǰ
	wheel_rpm[1] = (Mecanum_Data.Speed_ToChassis.vx - Mecanum_Data.Speed_ToChassis.vy -\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//��ǰ
	wheel_rpm[2] = (Mecanum_Data.Speed_ToChassis.vx + Mecanum_Data.Speed_ToChassis.vy -\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//���
	wheel_rpm[3] = -(Mecanum_Data.Speed_ToChassis.vx - Mecanum_Data.Speed_ToChassis.vy +\
					Mecanum_Data.Speed_ToChassis.vw *( MECANUM_LENGTH_A + MECANUM_LENGTH_B))* wheel_rpm_ratio;//�Һ�
    memcpy(Mecanum_Data.M3508_Setspeed,wheel_rpm, sizeof(wheel_rpm));
 }
 
 /**
  * @brief  ����̨�����µ��ٶ�ת��Ϊ���������µ��ٶ�
  * @param  angle ��̨����ڵ��̵ĽǶ�
  * @retval ƫ��ǣ��Ƕ���
  * @attention �ٶ������ٶ�������̨Ϊ����ϵ���˺����������ٶ�ת������������ϵ��
  */
void Mecanum_Absolute_Cal(fp32 angle)
{
    fp32 angle_hd = angle * PI / 180;

    Mecanum_Data.Speed_ToChassis.vw = Mecanum_Data.Speed_ToCloud.vw;
    Mecanum_Data.Speed_ToChassis.vx = Mecanum_Data.Speed_ToCloud.vx * cos(angle_hd) - \
                                      Mecanum_Data.Speed_ToCloud.vy * sin(angle_hd);
    Mecanum_Data.Speed_ToChassis.vy = Mecanum_Data.Speed_ToCloud.vx * sin(angle_hd) + \
                                      Mecanum_Data.Speed_ToCloud.vy * cos(angle_hd);

    //��֤�������������ͷ���ƶ���������ͷת��90��ʱx�����ٶȴ�1��0��
    //y�����ٶȴ�0��1����֤�Ӿ������������

    Mecanum_calc();
}

/**
  * @brief  ����3508Ŀ���ٶ�
  * @param  Motor ����ṹ��
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
  * @brief ��ȡ��������̨��ĽǶ�
* ��λ:��
  */
void Mecanum_GetAngle(fp32 angle)
{
    Mecanum_Data.Angle_ChassisToCloud = angle ;
}


/**
  * @brief  ���̵�����
  * @param  void
  * @retval void
  * @attention
  */
void Mecanum_Chassis_out()
{

	uint8_t data[8] = {0};
	Mecanum_Absolute_Cal(Mecanum_Data.Angle_ChassisToCloud); 	//������������Ŀ���ٶ�

	Mecanum_Set_Motor_Speed(M3508_Array); 						 //���ø��������Ŀ���ٶ�
	
	 /**************************����3508����ٶȻ�����*****************************/
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
	
   /***************************�������������͸����*******************************/
	
	M3508_FUN.M3508_setCurrent(M3508_Array[0].outCurrent,M3508_Array[1].outCurrent,
							   M3508_Array[2].outCurrent,M3508_Array[3].outCurrent,
							   data);
	Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle,&hfdcan1,FDCAN_STANDARD_ID,M3508_SENDID_Chassis,data);
}


/**
  * @brief  ����һ��������̨
  * @param  void
  * @retval void
  * @attention ��������ʹ�õ��̵�����������̨����
  */
void Mecanum_Chassis_Follow_Gimbal(void)
{
//	-298~62
//	-180~180
	fp32 angle_err = Mecanum_Data.Angle_ChassisToCloud;//��λ�Ƕȡ�
	if(angle_err < -180)
	angle_err+=360;	
	Mecanum_Data.Speed_ToCloud.vw = -1*Position_PID(&Chassis_Follow_Pid,0,angle_err);
	
}