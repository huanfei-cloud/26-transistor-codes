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

//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
M6020s_t M6020s_Yaw;                                    //ID为1
M6020s_t M6020s_Chassis1;
M6020s_t M6020s_Chassis2;
M6020s_t *M6020_Array[Totalnum] = {&M6020s_Yaw,&M6020s_Chassis1,&M6020s_Chassis2}; //对应电机的ID必须为：索引+1

#define M6020_Amount 1


M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit

/**
 * @brief motor﹍て
 * @param  motor结构体地址  
 * @param  设置的ID
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
  * @brief  设置M6020电机电压（id号为1~4）
  * @param  uqx (x:1~4) 对应id号电机的电压值，范围 -30000~0~30000
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
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //由0开始
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //解包数据，数据格式详见C620电调说明书P33
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
    //帧率统计，数据更新标志位
    M6020_Array[StdId]->InfoUpdateFrame++;
    M6020_Array[StdId]->InfoUpdateFlag = 1;

}

/**
  * @brief  设定M6020电机的目标角度
  * @param  M6020 	电机数据结构体地址 
  * @param  angle		机械角度值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
  * @retval None
  */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/**
  * @brief  M6020_Reset
  * @param  电机数据结构体地址
  * @retval None
  * 说明：调运此函数以解决totalAngle 等溢出的问题。
  */
void M6020_Reset(M6020s_t *m6020)
{
    //解包数据，数据格式详见C620电调说明书P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}


/**
 * @brief 电机速度目标值（输出电流值）更新
 * @param *motor 需要改变速度的电机地址
 * @param model 电机pid控制模式
 * @param target 速度目标值
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
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
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
			motor->outCurrent = (int16_t)Position_PID(&motor->v_pid_object,target, (float)motor->realSpeed);
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
 * @brief 电机位置目标值（设定速度值值）更新
 * @param *motor 需要改变位置的电机地址
 * @param model 电机pid控制模式
 * @param target 位置目标值
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
			motor->targetSpeed = (int16_t)Position_PID(&motor->l_pid_object,target,real);
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
			motor->targetSpeed = (int16_t)Position_PID(&motor->l_pid_object,target,real);
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
