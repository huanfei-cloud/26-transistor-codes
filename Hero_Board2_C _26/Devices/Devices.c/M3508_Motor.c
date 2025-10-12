/**
 * @file M3508_Motor.c
 * @author Why
 * @brief
 * @version 0.1
 * @date 2023-08-14
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "M3508_Motor.h"
#include <stdio.h>

/* M3508���е�������� */
M3508s_t M3508_Array[TotalNum] = {
    [Chassis_Left]    = {0},
    [Chassis_Forward] = {0},
    [Chassis_Right]   = {0},
    [Chassis_Back]    = {0},
    [FricL_Wheel]     = {0},
    [FricR_Wheel] 	  = {0},
    [Dial_Motor] 	  = {0},
};

M3508s_t M3508_Helm[4];

void M3508_getInfo(Can_Export_Data_t RxMessage);
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage);
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);

M3508_FUN_t M3508_FUN = M3508_FunGroundInit;
#undef M3508_FunGroundInit


/**
 * @brief motor?�l��
 * @param  motor�ṹ���ַ  
 * @param  ���õ�ID?}
 */
void M3508_Init(M3508s_t *motor, uint16_t _motor_id)
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
  * @brief  ����M3508�������ֵ��id��Ϊ1~4��
  * @param  iqx (x:1~4) ��Ӧid�ŵ���ĵ���ֵ����Χ-16384~0~16384
  * @retval None
  */
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data)
{
    //���ݸ�ʽ���˵����P32
    data[0] = iq1 >> 8;
    data[1] = iq1;
    data[2] = iq2 >> 8;
    data[3] = iq2;
    data[4] = iq3 >> 8;
    data[5] = iq3;
    data[6] = iq4 >> 8;
    data[7] = iq4;
}

/**
  * @brief  ��CAN�����л�ȡM3508�����Ϣ
  * @param  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */
void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508M_READID_START);
	  
    //������ݣ����ݸ�ʽ���C620���˵����P33
	  M3508_Helm[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508_Helm[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508_Helm[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508_Helm[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M3508_Helm[StdId].realAngle - M3508_Helm[StdId].lastAngle < -6000)
    {
        M3508_Helm[StdId].turnCount++;
    }
    else if (M3508_Helm[StdId].lastAngle - M3508_Helm[StdId].realAngle < -6000)
    {
        M3508_Helm[StdId].turnCount--;
    }
    M3508_Helm[StdId].totalAngle = M3508_Helm[StdId].realAngle + (8192 * M3508_Helm[StdId].turnCount);
    M3508_Helm[StdId].lastAngle = M3508_Helm[StdId].realAngle;

    //֡��ͳ�ƣ����ݸ��±�־λ
    M3508_Helm[StdId].InfoUpdateFrame++;
    M3508_Helm[StdId].InfoUpdateFlag = 1;

}

/**
 * @brief ������ֵת��ΪȦ��
 * @param ��������ֵ
 * @retval Ȧ��
 */
float encoder_to_circle(int32_t encoder)
{
    return (float)(encoder/8192);
}

/**
 * @brief Ȧ��ת��Ϊ��������ֵ
 * @param Ȧ��
 * @retval ��������ֵ
 */
int32_t circle_to_encoder(float circle)
{
    return (int32_t)(circle * 8192);
}


/**
 * @brief ����ٶ�Ŀ��ֵ���������ֵ������
 * @param *motor ��Ҫ�ı��ٶȵĵ����ַ
 * @param model ���pid����ģʽ
 * @param target �ٶ�Ŀ��ֵ
 */
void motor_velocity_change(M3508s_t *motor,pid_control model,CAN_HandleTypeDef *hcan,float target)
{
	if(hcan == &hcan1)
	{
    switch (motor->motor_id)
	{
	case (0x201):
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
		CAN1_0x200_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x200_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x202):
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
		CAN1_0x200_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x200_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x203):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[4] << 8 | CAN1_0x200_Tx_Data[5]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN1_0x200_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x200_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x204):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x200_Tx_Data[6] << 8 | CAN1_0x200_Tx_Data[7]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN1_0x200_Tx_Data[6] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x200_Tx_Data[7] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x205):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x1ff_Tx_Data[0] << 8 | CAN1_0x1ff_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x1ff_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x206):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x1ff_Tx_Data[2] << 8 | CAN1_0x1ff_Tx_Data[3]);

//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object, target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x1ff_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x207):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x1ff_Tx_Data[4] << 8 | CAN1_0x1ff_Tx_Data[5]);

//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->outCurrent = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_bullet_v, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x1ff_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x208):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x1ff_Tx_Data[6] << 8 | CAN1_0x1ff_Tx_Data[7]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}

		CAN1_0x1ff_Tx_Data[6] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x1ff_Tx_Data[7] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x209):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x2ff_Tx_Data[0] << 8 | CAN1_0x2ff_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->outCurrent = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_pitch_v, (float)motor->omega, target);
//		}

		CAN1_0x2ff_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x2ff_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x20A):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x2ff_Tx_Data[2] << 8 | CAN1_0x2ff_Tx_Data[3]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}

		CAN1_0x2ff_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x2ff_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x20B):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN1_0x2ff_Tx_Data[4] << 8 | CAN1_0x2ff_Tx_Data[5]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN1_0x2ff_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN1_0x2ff_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	}
}
	if(hcan == &hcan2)
	{
		switch (motor->motor_id)
	{
	case (0x201):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x200_Tx_Data[0] << 8 | CAN2_0x200_Tx_Data[1]);
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
		CAN2_0x200_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x200_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x202):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x200_Tx_Data[0] << 8 | CAN2_0x200_Tx_Data[1]);
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
		CAN2_0x200_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x200_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x203):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x200_Tx_Data[4] << 8 | CAN2_0x200_Tx_Data[5]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x200_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x200_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x204):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x200_Tx_Data[6] << 8 | CAN2_0x200_Tx_Data[7]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x200_Tx_Data[6] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x200_Tx_Data[7] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x205):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x1ff_Tx_Data[0] << 8 | CAN2_0x1ff_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
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
//			motor->outCurrent = (int16_t)(CAN2_0x1ff_Tx_Data[2] << 8 | CAN2_0x1ff_Tx_Data[3]);

//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object, target, (float)motor->realSpeed);
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
//			motor->outCurrent = (int16_t)(CAN2_0x1ff_Tx_Data[4] << 8 | CAN2_0x1ff_Tx_Data[5]);

//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->outCurrent = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_bullet_v, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x208):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x1ff_Tx_Data[6] << 8 | CAN2_0x1ff_Tx_Data[7]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}

		CAN2_0x1ff_Tx_Data[6] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x1ff_Tx_Data[7] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x209):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x2ff_Tx_Data[0] << 8 | CAN2_0x2ff_Tx_Data[1]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->outCurrent = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_pitch_v, (float)motor->omega, target);
//		}

		CAN2_0x2ff_Tx_Data[0] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x2ff_Tx_Data[1] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x20A):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x2ff_Tx_Data[2] << 8 | CAN2_0x2ff_Tx_Data[3]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}

		CAN2_0x2ff_Tx_Data[2] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x2ff_Tx_Data[3] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	case (0x20B):
	{
//		if (model == pid_control_increase)
//		{
//			motor->outCurrent = (int16_t)(CAN2_0x2ff_Tx_Data[4] << 8 | CAN2_0x2ff_Tx_Data[5]);
//			motor->outCurrent += (int16_t)Incremental_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
//		}
		if (model == pid_control_normal)
		{
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target,(float)motor->realSpeed);
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->outCurrent = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
//		}
		CAN2_0x2ff_Tx_Data[4] = ((uint16_t)motor->outCurrent) >> 8;
		CAN2_0x2ff_Tx_Data[5] = ((uint16_t)motor->outCurrent) & 0xff;
	}
	break;
	}
}
}
/**
 * @brief ���λ��Ŀ��ֵ���趨�ٶ�ֵֵ������
 * @param *motor ��Ҫ�ı�λ�õĵ����ַ
 * @param model ���pid����ģʽ
 * @param target λ��Ŀ��ֵ
 */
void motor_location_change(M3508s_t *motor,pid_control model,float target,float real)
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
			motor->targetSpeed = (int16_t)Position_PID(&motor->l_pid_object,circle_to_encoder(target),circle_to_encoder(real));
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->targetSpeed = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
//		}
		if (model == pid_control_angle)
		{
			motor->targetSpeed = (int16_t)Angle_PID(&motor->l_pid_object,circle_to_encoder(target),circle_to_encoder(real),8191);
    }
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
			motor->targetSpeed = (int16_t)Position_PID(&motor->l_pid_object,circle_to_encoder(target),circle_to_encoder(real));
		}
//		if (model == pid_control_frontfeed)
//		{
//			motor->targetSpeed = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
//		}
//		if (model == pid_control_frontfuzzy)
//		{
//			motor->targetSpeed = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
//		}
		if (model == pid_control_angle)
		{
			motor->targetSpeed = (int16_t)Angle_PID(&motor->l_pid_object,circle_to_encoder(target),circle_to_encoder(real),8191);
    }
    break;
	}
}
}

