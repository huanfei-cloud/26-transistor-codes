/**
 * @file chassis.c
 * @author  重构代码
 * @brief 控制底盘运动
 * @version 1.1
 * @date 2025-04-02
 * @copyright Transistor BUAA
 */
#include "steer_chassis.h"

/*常值数据定义*/
#define M3508_RATIO 19
#define DIR_GEAR_RATIO 7.47f
#define Radius 60
#define WHEEL_PERIMETER 376.98f
//四个驱动电机朝向前方时转向电机初始编码值
#define DIRMOTOR_LF_ANGLE 339.8731f
#define DIRMOTOR_LB_ANGLE 339.8731f
#define DIRMOTOR_RB_ANGLE 339.8731f
#define DIRMOTOR_RF_ANGLE 339.8731f
/*数据定义*/
Struct_CHASSIS_Manage_Object chassis_control;
int8_t dirt[4] = {1,-1,1,-1};

/**
 * @brief 角度范围的限制
 * @param 计算出的角度值
 * @param 最大值
 * @retval 限制后的角度值
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
 * @brief 底盘电机初始化
 * @param 结构体地址
 * @retval None
 */
void chassis_init(void)
{
    M3508_Init(&M3508_Helm[0],0x201);
	  M3508_Init(&M3508_Helm[1],0x202);
	  M3508_Init(&M3508_Helm[2],0x203);
  	M3508_Init(&M3508_Helm[3],0x204);
	  M3508_Init(&M3508_Helm[4],0x205);
	  M3508_Init(&M3508_Helm[5],0x206);
	  M3508_Init(&M3508_Helm[6],0x207);
  	M3508_Init(&M3508_Helm[7],0x208);
    
	  //驱动电机速度环初始化
		Position_PIDInit(&(M3508_Helm[0].v_pid_object),10.0f, 0.22f, 0,0, 16384,30000,6000);
	  Position_PIDInit(&(M3508_Helm[1].v_pid_object),10.0f, 0.22f, 0,0, 16384,30000,6000);
	  Position_PIDInit(&(M3508_Helm[2].v_pid_object),10.0f, 0.22f, 0,0, 16384,30000,6000);
	  Position_PIDInit(&(M3508_Helm[3].v_pid_object),10.0f, 0.22f, 0,0, 16384,30000,6000);
		//转向电机速度环初始化
	  Position_PIDInit(&(M3508_Helm[4].v_pid_object),1,0.1,1,0,700,30000,6000);
	  Position_PIDInit(&(M3508_Helm[5].v_pid_object),1,0.1,1,0,700,30000,6000);
	  Position_PIDInit(&(M3508_Helm[6].v_pid_object),1,0.1,1,0,700,30000,6000);
	  Position_PIDInit(&(M3508_Helm[7].v_pid_object),1,0.1,1,0,700,30000,6000);
		//转向电机位置环初始化
		Position_PIDInit(&(M3508_Helm[4].l_pid_object),0.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
		Position_PIDInit(&(M3508_Helm[5].l_pid_object),0.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
		Position_PIDInit(&(M3508_Helm[6].l_pid_object),0.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
		Position_PIDInit(&(M3508_Helm[7].l_pid_object),0.032f, 0.00001f, 0.05, 0, 30000, 10000 ,10000);
    
    M3508_Helm[0].targetSpeed = 0.0f;
		M3508_Helm[1].targetSpeed = 0.0f;
		M3508_Helm[2].targetSpeed = 0.0f;
		M3508_Helm[3].targetSpeed = 0.0f;
		M3508_Helm[4].targetSpeed = 0.0f;
		M3508_Helm[5].targetSpeed = 0.0f;
		M3508_Helm[6].targetSpeed = 0.0f;
		M3508_Helm[7].targetSpeed = 0.0f;

}   

/**
 * @brief 驱动电机速度设置
 * @param None
 * @retval None
 */
void move_motor_speed_set(void)
{
    fp32 wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f  / WHEEL_PERIMETER * M3508_RATIO ;

    chassis_control.motor_omega[0] = dirt[0] * sqrt( pow(((chassis_control.Speed_ToChassis).vy + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2) + pow(((chassis_control.Speed_ToChassis).vx - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;
    chassis_control.motor_omega[1] = dirt[1] * sqrt( pow(((chassis_control.Speed_ToChassis).vy - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2) + pow(((chassis_control.Speed_ToChassis).vx - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;
    chassis_control.motor_omega[2] = dirt[2] * sqrt( pow(((chassis_control.Speed_ToChassis).vy - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2) + pow(((chassis_control.Speed_ToChassis).vx + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;
    chassis_control.motor_omega[3] = dirt[3] * sqrt( pow(((chassis_control.Speed_ToChassis).vy + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2) + pow(((chassis_control.Speed_ToChassis).vx + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),2)) * wheel_rpm_ratio ;


}

/**
 * @brief 转向电机角度设置
 * @param None
 * @retval None
 */
void direction_motor_angle_set(void)
{
    fp64 atan_angle[4];
    fp64 error_angle[4];
	  fp64 finall_angle[4];
    if(!((chassis_control.Speed_ToCloud).vx == 0 && (chassis_control.Speed_ToCloud).vy == 0 && (chassis_control.Speed_ToCloud).wz == 0))
    {
        atan_angle[0] = atan2(((chassis_control.Speed_ToChassis).vx - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),((chassis_control.Speed_ToChassis).vy + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
        atan_angle[1] = atan2(((chassis_control.Speed_ToChassis).vx - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),((chassis_control.Speed_ToChassis).vy - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
        atan_angle[2] = atan2(((chassis_control.Speed_ToChassis).vx + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),((chassis_control.Speed_ToChassis).vy + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
        atan_angle[3] = atan2(((chassis_control.Speed_ToChassis).vx + (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f),((chassis_control.Speed_ToChassis).vy - (chassis_control.Speed_ToChassis).wz * Radius * 0.707107f)) * 180.0f / pi ;
    }
		else
		{
			 atan_angle[0] = 0.0f;
			 atan_angle[1] = 0.0f;
			 atan_angle[2] = 0.0f;
			 atan_angle[3] = 0.0f;
		} 
       finall_angle[0] = atan_angle[0] + DIRMOTOR_LF_ANGLE;
		   finall_angle[1] = atan_angle[1] + DIRMOTOR_LB_ANGLE;
		   finall_angle[2] = atan_angle[2] + DIRMOTOR_RF_ANGLE;
		   finall_angle[3] = atan_angle[3] + DIRMOTOR_RB_ANGLE;
		if(finall_angle[0]>360)
		{
			finall_angle[0] -= 360;
		}
		else if(finall_angle[0]<0.0f)
		{
			finall_angle[0] += 360;
		}
		if(finall_angle[1]>8192.0f)
		{
			finall_angle[1] -= 360;
		}
		else if(finall_angle[1]<0.0f)
		{
			finall_angle[1] += 360;
		}
		if(finall_angle[2]>8192.0f)
		{
			finall_angle[2] -= 360;
		}
		else if(finall_angle[2]<0.0f)
		{
			finall_angle[2] += 360;
		}			
		if(finall_angle[3]>8192.0f)
		{
			finall_angle[3] -= 360;
		}
		else if(finall_angle[3]<0.0f)
		{
			finall_angle[3] += 360;
		}		
        error_angle[0] =Angle_Limit((finall_angle[0] - MA600s[0].Angle),180.0f);
		    error_angle[1] =Angle_Limit((finall_angle[1] - MA600s[1].Angle),180.0f);
		    error_angle[2] =Angle_Limit((finall_angle[2] - MA600s[2].Angle),180.0f);
		    error_angle[3] =Angle_Limit((finall_angle[3] - MA600s[3].Angle),180.0f);
		 	
		if(error_angle[0]>90.0f || error_angle[0]<-90.0f)
    {
        dirt[0] = -1;
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
        dirt[0] = 1;
    }
    if(error_angle[1]>90.0f || error_angle[1]<-90.0f)
    {
        dirt[1] = 1;
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
        dirt[1] = -1;
    }
    if(error_angle[2]>90.0f || error_angle[2]<-90.0f)
    {
        dirt[2] = -1;
			if(error_angle[2] > 90.0f)
				{
            finall_angle[2] = finall_angle[2] - 180.0f;
				}
				else if(error_angle[0] < -90.0f)
				{
					  finall_angle[2] = finall_angle[2] + 180.0f;
				}
    }
    else
    {
        dirt[2] = 1;
    }
    if(error_angle[3]>90.0f || error_angle[3]<-90.0f)
    {
        dirt[3] = 1;
			if(error_angle[3] > 90.0f)
				{
            finall_angle[3] = finall_angle[3] - 180.0f;
				}
				else if(error_angle[3] < -90.0f)
				{
					  finall_angle[3] = finall_angle[3] + 180.0f;
				}
     }
    else
    {
        dirt[3] = -1;
    }
    chassis_control.motor_location[0] = finall_angle[0] / 360.0f;
		chassis_control.motor_location[1] = finall_angle[1] / 360.0f;
		chassis_control.motor_location[2] = finall_angle[2] / 360.0f;
		chassis_control.motor_location[3] = finall_angle[3] / 360.0f;
}

/**
 * @brief 将云台坐标系下的速度转换为底盘坐标系下的速度
 * @param angle 云台相对于底盘的角度
 * @retval 
 */
void v_cloud_convertto_chassis(fp32 angle)
{
    fp32 angle_hd = angle *pi / 180;

 chassis_control.Speed_ToChassis.vx = chassis_control.Speed_ToCloud.vx * cos(angle_hd) - chassis_control.Speed_ToCloud.vy * sin(angle_hd);
 chassis_control.Speed_ToChassis.vy = chassis_control.Speed_ToCloud.vx * sin(angle_hd) + chassis_control.Speed_ToCloud.vy * cos(angle_hd);
 chassis_control.Speed_ToChassis.wz = chassis_control.Speed_ToCloud.wz;
}


/**
 * @brief 底盘目标值计算函数
 * @param None
 * @retval None
 */
void chassis_target_calc(void)
{
    v_cloud_convertto_chassis(chassis_control.Angle_ChassisToCloud);
    move_motor_speed_set();
    direction_motor_angle_set();
	//转向电机目标位置设置
    M3508_Helm[4].targetLocation = chassis_control.motor_location[0];
    M3508_Helm[5].targetLocation = chassis_control.motor_location[1];
    M3508_Helm[6].targetLocation = chassis_control.motor_location[2];
    M3508_Helm[7].targetLocation = chassis_control.motor_location[3];
	if( M3508_Helm[7].targetLocation<0)
	{
		 M3508_Helm[7].targetLocation+=1.0f;
	}
	else if( M3508_Helm[7].targetLocation>1.0f)
	{
		 M3508_Helm[7].targetLocation-=1.0f;
	}
	if( M3508_Helm[6].targetLocation<0)
	{
		 M3508_Helm[6].targetLocation+=1.0f;
	}
	else if( M3508_Helm[6].targetLocation>1.0f)
	{
		 M3508_Helm[6].targetLocation-=1.0f;
	}if( M3508_Helm[5].targetLocation<0)
	{
		 M3508_Helm[5].targetLocation+=1.0f;
	}
	else if( M3508_Helm[5].targetLocation>1.0f)
	{
		 M3508_Helm[5].targetLocation-=1.0f;
	}if( M3508_Helm[4].targetLocation<0)
	{
		 M3508_Helm[4].targetLocation+=1.0f;
	}
	else if( M3508_Helm[4].targetLocation>1.0f)
	{
		 M3508_Helm[7].targetLocation-=1.0f;
	}
    //驱动电机目标速度设置
    M3508_Helm[0].targetSpeed = chassis_control.motor_omega[0];
    M3508_Helm[1].targetSpeed = chassis_control.motor_omega[1];
    M3508_Helm[2].targetSpeed = chassis_control.motor_omega[2];
    M3508_Helm[3].targetSpeed = chassis_control.motor_omega[3];
 
}  

/**
 * @brief 电机电流输出函数
 * @param None
 * @retval None
 */
void steer_chassis_out(void)
{
    chassis_target_calc();
	
	  motor_location_change(&M3508_Helm[4],pid_control_normal,M3508_Helm[4].targetLocation,MA600s[0].Circle);
	  motor_location_change(&M3508_Helm[5],pid_control_normal,M3508_Helm[5].targetLocation,MA600s[1].Circle);
	  motor_location_change(&M3508_Helm[6],pid_control_normal,M3508_Helm[6].targetLocation,MA600s[2].Circle);
	  motor_location_change(&M3508_Helm[7],pid_control_normal,M3508_Helm[7].targetLocation,MA600s[3].Circle);

    motor_velocity_change(&M3508_Helm[0],pid_control_normal,&hcan1,M3508_Helm[0].targetSpeed);
	  motor_velocity_change(&M3508_Helm[1],pid_control_normal,&hcan1,M3508_Helm[1].targetSpeed);
		motor_velocity_change(&M3508_Helm[2],pid_control_normal,&hcan1,M3508_Helm[2].targetSpeed);
		motor_velocity_change(&M3508_Helm[3],pid_control_normal,&hcan1,M3508_Helm[3].targetSpeed);
		motor_velocity_change(&M3508_Helm[4],pid_control_normal,&hcan2,M3508_Helm[4].targetSpeed);
		motor_velocity_change(&M3508_Helm[5],pid_control_normal,&hcan2,M3508_Helm[5].targetSpeed);
		motor_velocity_change(&M3508_Helm[6],pid_control_normal,&hcan2,M3508_Helm[6].targetSpeed);
		motor_velocity_change(&M3508_Helm[7],pid_control_normal,&hcan2,M3508_Helm[7].targetSpeed);


    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan1,CAN_ID_STD,0x200,CAN1_0x200_Tx_Data);
    Can_Fun.CAN_SendData(CAN_SendHandle,&hcan2,CAN_ID_STD,0x1ff,CAN2_0x1ff_Tx_Data);
}


void steer_getangle(fp32 angle)
{
    chassis_control.Angle_ChassisToCloud = angle  ;
}
