/**
 * @file M6020_Motor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "M6020_Motor.h"

//ֱ��������Ӧ�ĵ���Ľṹ����������飬ֱ�۱��ں��ڵ��Թ۲�����ʹ�á�
M6020s_t M6020s_Yaw;                                    //IDΪ1
M6020s_t M6020s_Chassis1;
M6020s_t M6020s_Chassis2;
M6020s_t *M6020_Array[Totalnum] = {&M6020s_Yaw,&M6020s_Chassis1,&M6020s_Chassis2}; //��Ӧ�����ID����Ϊ������+1

#define M6020_Amount 1


M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit

/**
 * @brief motor��l��
 * @param  motor�ṹ����?  
 * @param  ���õ�ID�}
 */
void M6020_Init(M6020s_t *motor, uint16_t _motor_id)
{
	motor->motor_id = _motor_id;
	motor->realSpeed = 0;
	motor->temperture = 0;
	motor->realAngle = 0;
	motor->realCurrent = 0;

	motor->lastAngle = 0;
	motor->totalAngle = 0;
	motor->turnCount = 0;

  motor->outCurrent = 0;

	motor->InfoUpdateFlag = 0;
}

/**
  * @brief  ����M6020�����ѹ��id��Ϊ1~4��
  * @param  uqx (x:1~4) ��Ӧid�ŵ���ĵ�ѹֵ�����? -30000~0~30000
  * @retval None
  */
 void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data)
 {


 	data[0] = uq1 >> 8;
 	data[1] = uq1;
 	data[2] = uq2 >> 8;
 	data[3] = uq2;
 	data[4] = uq3 >> 8;
 	data[5] = uq3;
 	data[6] = uq4 >> 8;
 	data[7] = uq4;

// 	// CAN_SendData(&hcan1, CAN_ID_STD, M6020_SENDID, data);
 }

/**
  * @brief  ��CAN�����л�ȡM6020������?
  * @param  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //��0��ʼ
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //������ݣ����ݸ�ʽ���C620���˵����P33
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -6500)
    {
        M6020_Array[StdId]->turnCount++;
    }

    if (M6020_Array[StdId]->lastAngle - M6020_Array[StdId]->realAngle < -6500)
    {
        M6020_Array[StdId]->turnCount--;
    }

    M6020_Array[StdId]->totalAngle = M6020_Array[StdId]->realAngle + (8192 * M6020_Array[StdId]->turnCount);
    //֡��ͳ�ƣ����ݸ��±�־λ
    M6020_Array[StdId]->InfoUpdateFrame++;
    M6020_Array[StdId]->InfoUpdateFlag = 1;

}

/**
  * @brief  �趨M6020�����Ŀ��Ƕ�
  * @param  M6020 	������ݽṹ���ַ 
  * @param  angle		��е�Ƕ�ֵ����Χ 0~8191 ��������0��8191�ᵼ�µ���񵴣��?�����޷�
  * @retval None
  */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/**
  * @brief  M6020_Reset
  * @param  ������ݽṹ���ַ
  * @retval None
  * ˵�������˴˺����Խ��totalAngle �����������?
  */
void M6020_Reset(M6020s_t *m6020)
{
    //������ݣ����ݸ�ʽ���C620���˵����P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}


/**
 * @brief ����ٶ�Ŀ��ֵ���������ֵ������
 * @param *motor ��Ҫ�ı��ٶȵĵ�����?
 * @param model ���pid����ģʽ
 * @param target �ٶ�Ŀ��ֵ
 */
void M6020_velocity_change(M6020s_t *motor,pid_control model,CAN_HandleTypeDef *hcan,float target)
{
	if(hcan == &hcan2)
	{
    switch (motor->motor_id)
	{
	case (0x205):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[0] << 8 | CAN1_0x200_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)speed_angle_limit_pid(&motor->v_pid_object,target, (float)motor->realSpeed,motor->realAngle);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x206):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[0] << 8 | CAN1_0x200_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)speed_angle_limit_pid(&motor->v_pid_object,target, (float)motor->realSpeed,motor->realAngle);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x207):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[0] << 8 | CAN1_0x200_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x208):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[0] << 8 | CAN1_0x200_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[6] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[7] = ((uint16_t)motor->outCurrent) & 0xff;
	  }
	  break;
}
}
}
	
/**
 * @brief ���λ��Ŀ��ֵ����?�ٶ�ֵֵ������
 * @param *motor ��Ҫ�ı�λ�õĵ�����?
 * @param model ���pid����ģʽ
 * @param target λ��Ŀ��ֵ
 */
void M6020_location_change(M6020s_t *motor,pid_control model,int16_t target,int16_t real)
{
    switch (motor->motor_id)
	{
	  case (0x201):
    case (0x202):
    case (0x203):
    case (0x204):
	{
//		if (model == pid_control_increase)
//		{
//			motor->targetSpeed += (int16_t)Incremental_PID(&motor->l_pid_object,circle_to_encoder(target),motor->totalAngle);
//		}
		if (model == pid_control_normal)
		{
			motor->targetSpeed = (int16_t)Angle_PID(&motor->l_pid_object,target,real,8191);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->targetSpeed = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
//		}
	}
	break;
	  case (0x205):
    case (0x206):
    case (0x207):
    case (0x208):
    {
//        if (model == pid_control_increase)
//		{
//			motor->targetSpeed += (int16_t)Incremental_PID(&motor->l_pid_object,circle_to_encoder(target),motor->totalAngle);
//		}
		if (model == pid_control_normal)
		{
			motor->targetSpeed = (int16_t)Angle_PID(&motor->l_pid_object,target,real,8191);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->targetSpeed = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->targetSpeed = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
//		}
    }
    break;
	}
}
