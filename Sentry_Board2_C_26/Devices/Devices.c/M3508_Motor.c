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

/* M3508所有电机的数组 */
M3508s_t M3508_Array[TotalNum] = {
    [Chassis_Left]    = {0},
    [Chassis_Forward] = {0},
    [Chassis_Right]   = {0},
    [Chassis_Back]    = {0},
    [FricL_Wheel]     = {0},
    [FricR_Wheel] 	  = {0},
    [Dial_Motor] 	  = {0},
};


void M3508_getInfo(Can_Export_Data_t RxMessage);
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage);
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data);

M3508_FUN_t M3508_FUN = M3508_FunGroundInit;
#undef M3508_FunGroundInit


/**
 * @brief motor﹍て
 * @param  motor结构体地址  
 * @param  设置的ID
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
  * @brief  设置M3508电机电流值（id号为1~4）
  * @param  iqx (x:1~4) 对应id号电机的电流值，范围-16384~0~16384
  * @retval None
  */
void M3508_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4, uint8_t *data)
{
    //数据格式详见说明书P32
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
  * @brief  从CAN报文中获取M3508电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508M_READID_START);
	  
    //解包数据，数据格式详见C620电调说明书P33
	  M3508_Array[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508_Array[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508_Array[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508_Array[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M3508_Array[StdId].realAngle - M3508_Array[StdId].lastAngle < -6000)
    {
        M3508_Array[StdId].turnCount++;
    }
    else if (M3508_Array[StdId].lastAngle - M3508_Array[StdId].realAngle < -6000)
    {
        M3508_Array[StdId].turnCount--;
    }
    M3508_Array[StdId].totalAngle = M3508_Array[StdId].realAngle + (8192 * M3508_Array[StdId].turnCount);
    M3508_Array[StdId].lastAngle = M3508_Array[StdId].realAngle;

    //帧率统计，数据更新标志位
    M3508_Array[StdId].InfoUpdateFrame++;
    M3508_Array[StdId].InfoUpdateFlag = 1;

}

/**
 * @brief 编码数值转化为圈数
 * @param 编码器数值
 * @retval 圈数
 */
float encoder_to_circle(int32_t encoder)
{
    return (float)(encoder/8192);
}

/**
 * @brief 圈数转换为编码器数值
 * @param 圈数
 * @retval 编码器数值
 */
int32_t circle_to_encoder(float circle)
{
    return (int32_t)(circle * 8192);
}


/**
 * @brief 电机速度目标值（输出电流值）更新
 * @param *motor 需要改变速度的电机地址
 * @param model 电机pid控制模式
 * @param target 速度目标值
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
 * @brief 电机位置目标值（设定速度值值）更新
 * @param *motor 需要改变位置的电机地址
 * @param model 电机pid控制模式
 * @param target 位置目标值
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
    }
    break;
	}
}

