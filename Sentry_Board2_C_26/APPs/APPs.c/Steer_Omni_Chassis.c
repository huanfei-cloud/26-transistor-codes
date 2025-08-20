/**
 * @file Steer_Omni_Chassis.c
 * @author xhf
 * @brief
 * @version 0.1
 * @date 2025-08-15
 * @copyright
 */

#include "Steer_Omni_Chassis.h"

/********��������********/
Steer_Omni_Data_t Steer_Omni_Data;
int8_t dirt[2] = {-1,1};
positionpid_t chassis_follow;
int flag =0;
/**
 * @brief �Ƕȷ�Χ������
 * @param ������ĽǶ�ֵ
 * @param ���ֵ
 * @retval ���ƺ�ĽǶ�ֵ
 */
fp64 Angle_Limit(fp64 angle,fp64 max)
{
    if(angle >max)
    {
        angle -= max; 
    }
    else if (angle < -max)
    {
        angle += max;
    }
    return angle;
} 

/**
 * @brief ���̵����ʼ��
 * @param �ṹ���ַ
 * @retval None
 */
void Chassis_Init(void)
{
    M3508_Init(&M3508_Array[0],0x201);
	  M3508_Init(&M3508_Array[1],0x202);
    M3508_Init(&M3508_Array[2],0x203);
	  M3508_Init(&M3508_Array[3],0x204);
    M6020_Init(&M6020s_Chassis1,0x206);
    M6020_Init(&M6020s_Chassis2,0x207);

    //��������ٶȻ���ʼ��
    Position_PIDInit(&(M3508_Array[0].v_pid_object),10.0f, 0.22f, 0,0,800,30000,6000);
		Position_PIDInit(&(M3508_Array[1].v_pid_object),10.0f, 0.22f, 0,0,800,30000,6000);
		Position_PIDInit(&(M3508_Array[2].v_pid_object),10.0f, 0.22f, 0,0,800,30000,6000);
		Position_PIDInit(&(M3508_Array[3].v_pid_object),10.0f, 0.22f, 0,0,800,30000,6000);
    //ת�����ٶȻ���ʼ��
  	Position_PIDInit(&(M6020s_Chassis1.v_pid_object),250,0.1,150,0,7000,30000,6000);
    Position_PIDInit(&(M6020s_Chassis2.v_pid_object),250,0.1,150,0,7000,30000,6000);
    //ת����λ�û���ʼ��
		Position_PIDInit(&(M6020s_Chassis1.l_pid_object),2.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
		Position_PIDInit(&(M6020s_Chassis2.l_pid_object),2.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
    Position_PIDInit(&(chassis_follow), 0.5f, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 10000.0f);
    M3508_Array[0].targetSpeed = 0.0f;
    M3508_Array[1].targetSpeed = 0.0f;
    M3508_Array[2].targetSpeed = 0.0f;
    M3508_Array[3].targetSpeed = 0.0f;

    M6020s_Chassis1.targetAngle = DIRMOTOR_LB_ANGLE;
    M6020s_Chassis2.targetAngle = DIRMOTOR_RB_ANGLE;

}

/**
 * @brief ����̨����ϵ�µ��ٶ�ת��Ϊ��������ϵ�µ��ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
 * @retval 
 */
void v_cloud_convertto_chassis(fp32 angle)
{
    fp32 angle_hd = angle *pi / 180;

    Steer_Omni_Data.Speed_ToChassis.vx =Steer_Omni_Data.Speed_ToCloud.vx * cos(angle_hd) -Steer_Omni_Data.Speed_ToCloud.vy * sin(angle_hd);
    Steer_Omni_Data.Speed_ToChassis.vy =Steer_Omni_Data.Speed_ToCloud.vx * sin(angle_hd) +Steer_Omni_Data.Speed_ToCloud.vy * cos(angle_hd);
    Steer_Omni_Data.Speed_ToChassis.wz =Steer_Omni_Data.Speed_ToCloud.wz;

}
/**
 * @brief ���̸���ģʽ
 * @param angle ��̨����ڵ��̵ĽǶ�
 * @param ����Ϊ1
 * @param kp
 * @retval ????????????????
 */
void chassis_follow_mode(float angle, uint8_t start_flag)
{
    if(start_flag)
    {
        if(abs(angle)<10)
        {
            return ;
        }
        Steer_Omni_Data.Speed_ToChassis.wz +=Position_PID(&chassis_follow,angle, 0);
    }
}
/**
 * @brief ת�����Ƕ�����
 * @param None
 * @retval None
 */
void direction_motor_angle_set(void)
{
    fp64 atan_angle[2];
    fp64 error_angle[2];
    fp64 finall_angle[2];

    if(!((Steer_Omni_Data.Speed_ToCloud).vx == 0 && (Steer_Omni_Data.Speed_ToCloud).vy == 0 && (Steer_Omni_Data.Speed_ToCloud).wz == 0))
    {
        atan_angle[0] = atan2(((Steer_Omni_Data.Speed_ToChassis).vx - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
        atan_angle[1] = atan2(((Steer_Omni_Data.Speed_ToChassis).vx + (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),((Steer_Omni_Data.Speed_ToChassis).vy + (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;        
    }
    else
    {
        atan_angle[0] = 0.0f;
        atan_angle[1] = 0.0f;
    }

    finall_angle[0] = atan_angle[0] + DIRMOTOR_LB_ANGLE / 8192 * 360.0f;
	  finall_angle[1] = atan_angle[1] + DIRMOTOR_RB_ANGLE / 8192 * 360.0f;
		
		

    error_angle[0] =Angle_Limit((finall_angle[0] - (M6020s_Chassis1.realAngle /8192 * 360.0f)),180.0f);
	  error_angle[1] =Angle_Limit((finall_angle[1] - (M6020s_Chassis2.realAngle /8192 * 360.0f)),180.0f);

    if(error_angle[0]>90.0f || error_angle[0]<-90.0f)
    {
        dirt[0] = 1;
        if(error_angle[0] > 90.0f)
				{
            finall_angle[0] = finall_angle[0] - 180.0f;
				}
				else if(error_angle[0] < -90.0f)
				{
					finall_angle[0] = finall_angle[0] + 180.0f;
				}
    }
    else
    {
        dirt[0] = -1;
    }
    if(error_angle[1]>90.0f || error_angle[1]<-90.0f)
    {
        dirt[1] = -1;
			if(error_angle[1] > 90.0f)
				{
            finall_angle[1] = finall_angle[1] - 180.0f;
				}
				else if(error_angle[1] < -90.0f)
				{
					finall_angle[1] = finall_angle[1] + 180.0f;
				}
    }
    else
    {
        dirt[1] = 1;
    }

    Steer_Omni_Data.M6020_Setposition[0] = finall_angle[0] * 8192 / 360.0f;
    Steer_Omni_Data.M6020_Setposition[1] = finall_angle[1] * 8192 / 360.0f;

}

/**
 * @brief ��������ٶ�����
 * @param None
 * @retval None
 */
void move_motor_speed_set(void)
{
    fp32 wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f  / WHEEL_PERIMETER * M3508_RATIO ;

    // ��ǰ���ٶȼ��㣺
    Steer_Omni_Data.M3508_Setspeed[0] = ((Steer_Omni_Data.Speed_ToChassis.vx-Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_y)*cos(lf_omni_angle/180*pi)+
                    (Steer_Omni_Data.Speed_ToChassis.vy+Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_x)*sin(lf_omni_angle/180*pi)) * wheel_rpm_ratio; 
    //������ٶȼ���
    Steer_Omni_Data.M3508_Setspeed[1] = dirt[0] * sqrt( pow(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),2) + 
                    pow(((Steer_Omni_Data.Speed_ToChassis).vx - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;
    //�Һ����ٶȼ���
    Steer_Omni_Data.M3508_Setspeed[2] = dirt[1] * sqrt( pow(((Steer_Omni_Data.Speed_ToChassis).vy - (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),2) + 
                    pow(((Steer_Omni_Data.Speed_ToChassis).vx + (Steer_Omni_Data.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;
    // ��ǰ���ٶȼ��㣺
    Steer_Omni_Data.M3508_Setspeed[3] =((Steer_Omni_Data.Speed_ToChassis.vx-Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_y)*cos(rf_omni_angle/180*pi)+
                    (Steer_Omni_Data.Speed_ToChassis.vy+Steer_Omni_Data.Speed_ToChassis.wz*Length_wheel_x)*sin(rf_omni_angle/180*pi)) * wheel_rpm_ratio; 

}

/**
 * @brief ����Ŀ��ֵ���㺯��
 * @param None
 * @retval None
 */
void chassis_target_calc(void)
{
    v_cloud_convertto_chassis(Steer_Omni_Data.Angle_ChassisToCloud);
    chassis_follow_mode(Steer_Omni_Data.Angle_ChassisToCloud,flag);
    direction_motor_angle_set();
    move_motor_speed_set();
    //ת����Ŀ��λ������
    M6020s_Chassis1.targetAngle = Steer_Omni_Data.M6020_Setposition[0];
    M6020s_Chassis2.targetAngle = Steer_Omni_Data.M6020_Setposition[1];
	  if( M6020s_Chassis1.targetAngle < 0 )
		{
			M6020s_Chassis1.targetAngle += 8192 ; 
		}
		else if ( M6020s_Chassis1.targetAngle > 8192 )
		{
			M6020s_Chassis1.targetAngle -= 8192 ;
		}
		if( M6020s_Chassis2.targetAngle < 0 )
		{
			M6020s_Chassis2.targetAngle += 8192 ; 
		}
		else if ( M6020s_Chassis2.targetAngle > 8192 )
		{
			M6020s_Chassis2.targetAngle -= 8192 ;
		}
    //�������Ŀ���ٶ�����
    M3508_Array[0].targetSpeed = Steer_Omni_Data.M3508_Setspeed[0];
    M3508_Array[1].targetSpeed = Steer_Omni_Data.M3508_Setspeed[1];
    M3508_Array[2].targetSpeed = Steer_Omni_Data.M3508_Setspeed[2];
    M3508_Array[3].targetSpeed = Steer_Omni_Data.M3508_Setspeed[3];

}

/**
 * @brief ��������������
 * @param None
 * @retval None
 */
void Steer_Omni_Chassis_Out(void)
{
    chassis_target_calc();
    
    M6020_location_change(&M6020s_Chassis1,pid_control_normal,M6020s_Chassis1.targetAngle,M6020s_Chassis1.totalAngle);
    M6020_location_change(&M6020s_Chassis2,pid_control_normal,M6020s_Chassis2.targetAngle,M6020s_Chassis2.totalAngle);

    M6020_velocity_change(&M6020s_Chassis1,pid_control_normal,&hcan2,M6020s_Chassis1.targetSpeed);
    M6020_velocity_change(&M6020s_Chassis2,pid_control_normal,&hcan2,M6020s_Chassis2.targetSpeed);

    motor_velocity_change(&M3508_Array[0],pid_control_normal,&hcan1,M3508_Array[0].targetSpeed);
    motor_velocity_change(&M3508_Array[1],pid_control_normal,&hcan1,M3508_Array[1].targetSpeed);
    motor_velocity_change(&M3508_Array[2],pid_control_normal,&hcan1,M3508_Array[2].targetSpeed);
    motor_velocity_change(&M3508_Array[3],pid_control_normal,&hcan1,M3508_Array[3].targetSpeed);

    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan1,CAN_ID_STD,0x200,CAN1_0x200_Tx_Data);
    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan2,CAN_ID_STD,0x1ff,CAN2_0x1ff_Tx_Data);

}

/**
 * @brief ���ݵ�����̨����ԽǶ�
 * @param None
 * @retval None
 */
void Steer_Omni_GetAngle(fp32 angle)
{
	Steer_Omni_Data.Angle_ChassisToCloud = angle;
}
