/**
 * @file bsp_motor.c
 * @author xyz
 * @brief motor相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#include "DJ_Motor.h"

/********变量定义********/
DJ_Motors_t M2006s_J2;
DJ_Motors_t M2006s_J3;

/**
 * @brief motor初始化
 * @param 电机处理结构体
 * @param id(默认电机1对应电调ID1，对应pid结构体1
 */
void motor_init(DJ_Motors_t *motor, uint16_t _motor_id)
{
	motor->can_id = _motor_id;
	motor->omega = 0;
	motor->temperaure = 0;
	motor->encoder = 0;
	motor->torque = 0;

	motor->i_send = 0;
	motor->pre_encoder = 0;
	motor->now_encoder = 0;
	motor->delta_encoder = 0;
	motor->total_encoder = 0;
	motor->total_round = 0;
	motor->zero_flag=0;
	motor->UpdateFlag=0;
}

/**
 * @brief 机械角度转化为圈数
 * @param encoder 机械角度
 * @param circle 圈数
 */

float encoder_to_circle(int32_t encoder)
{

	return ((float)encoder / 8192);
}

/**
 * @brief 圈数转化为机械角度
 * @param circle 圈数
 * @param encoder 机械角度 
 */
int32_t circle_to_encoder(float circle)
{
	return (int32_t)(circle * 8192);
}

/**
* @brief 获得电机的相关数据
 * @param *motor 要用pid控制的电机的地址
 * @param *data  数据的地址
 * @retval None
 */

void get_motor_data(DJ_Motors_t *motor,Can_Export_Data_t RxMessage)
{
	motor->encoder = (uint16_t)RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1];
	motor->omega = (int16_t)RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3];
	motor->torque = (int16_t)RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5];
	motor->temperaure = (uint16_t)RxMessage.CANx_Export_RxMessage[6] << 8;
	
	motor->now_encoder = motor->encoder;
	motor->delta_encoder = (int32_t)(motor->now_encoder - motor->pre_encoder);
	
	if(motor->zero_flag==1)//必须保证pre_encoder不为0才能判断圈数
	{
		if (motor->delta_encoder < -4096)
		{
			// 编码器正向旋转了一圈
			motor->total_round++;
		}
		if (motor->delta_encoder > 4096)
		{
			// 编码器逆向旋转了一圈
			motor->total_round--;
		}

	}
		if(motor->zero_flag==0)
		{
			motor->zero_encoder=motor->encoder;
			motor->zero_flag=1;
		}	
		motor->total_encoder = (motor->total_round) * 8192 + (int32_t)motor->now_encoder - motor->zero_encoder;
		motor->pre_encoder = motor->now_encoder;
}

/**
 *@brief 电机速度值的改变
 * @param *motor 要用pid控制的电机的地址
 * @param  model pid控制方式.
 * @param  target 目标速度值
 */
void DJmotor_velocity_change(DJ_Motors_t *motor,CAN_HandleTypeDef *hcan,enum pid_control model, float target)
{
	if (hcan == &hcan1)
	{
	switch (motor->can_id)
	{
	case (0x201):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x200_Tx_Data[0] << 8 | CAN1_0x200_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN1_0x200_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x200_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x202):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x200_Tx_Data[2] << 8 | CAN1_0x200_Tx_Data[3]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if(motor->i_send > 10000)
		{
			motor->i_send = 10000;
		}
		else if(motor->i_send < -10000)
		{
			motor->i_send = -10000;
		}
		CAN1_0x200_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x200_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x203):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x200_Tx_Data[4] << 8 | CAN1_0x200_Tx_Data[5]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if(motor->i_send > 10000)
		{
			motor->i_send = 10000;
		}
		else if(motor->i_send < -10000)
		{
			motor->i_send = -10000;
		}
		CAN1_0x200_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x200_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x204):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x200_Tx_Data[6] << 8 | CAN1_0x200_Tx_Data[7]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN1_0x200_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x200_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x205):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x1ff_Tx_Data[0] << 8 | CAN1_0x1ff_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_shoot_l, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x1ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x206):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x1ff_Tx_Data[2] << 8 | CAN1_0x1ff_Tx_Data[3]);

			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_shoot_r, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x1ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x207):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x1ff_Tx_Data[4] << 8 | CAN1_0x1ff_Tx_Data[5]);

			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_bullet_v, (float)motor->omega, target);
//		}
		CAN1_0x1ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x1ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x208):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x1ff_Tx_Data[6] << 8 | CAN1_0x1ff_Tx_Data[7]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}

		CAN1_0x1ff_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x1ff_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x209):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x2ff_Tx_Data[0] << 8 | CAN1_0x2ff_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_pitch_v, (float)motor->omega, target);
//		}

		CAN1_0x2ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x2ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x20A):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x2ff_Tx_Data[2] << 8 | CAN1_0x2ff_Tx_Data[3]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}

		CAN1_0x2ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x2ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x20B):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN1_0x2ff_Tx_Data[4] << 8 | CAN1_0x2ff_Tx_Data[5]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN1_0x2ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN1_0x2ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	}
}
	else if (hcan == &hcan2)
	{
	switch (motor->can_id)
	{
	case (0x201):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x200_Tx_Data[0] << 8 | CAN2_0x200_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN2_0x200_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x200_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x202):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x200_Tx_Data[2] << 8 | CAN2_0x200_Tx_Data[3]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if(motor->i_send > 10000)
		{
			motor->i_send = 10000;
		}
		else if(motor->i_send < -10000)
		{
			motor->i_send = -10000;
		}
		CAN2_0x200_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x200_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x203):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x200_Tx_Data[4] << 8 | CAN2_0x200_Tx_Data[5]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if(motor->i_send > 10000)
		{
			motor->i_send = 10000;
		}
		else if(motor->i_send < -10000)
		{
			motor->i_send = -10000;
		}
		CAN2_0x200_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x200_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x204):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x200_Tx_Data[6] << 8 | CAN2_0x200_Tx_Data[7]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN2_0x200_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x200_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x205):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x1ff_Tx_Data[0] << 8 | CAN2_0x1ff_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_shoot_l, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x1ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x206):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x1ff_Tx_Data[2] << 8 | CAN2_0x1ff_Tx_Data[3]);

			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_shoot_r, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x1ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x207):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x1ff_Tx_Data[4] << 8 | CAN2_0x1ff_Tx_Data[5]);

			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_bullet_v, (float)motor->omega, target);
//		}
		CAN2_0x1ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x1ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x208):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x1ff_Tx_Data[6] << 8 | CAN2_0x1ff_Tx_Data[7]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}

		CAN2_0x1ff_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x1ff_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x209):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x2ff_Tx_Data[0] << 8 | CAN2_0x2ff_Tx_Data[1]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
//		if (model == pid_control4)
//		{
//			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_pitch_v, (float)motor->omega, target);
//		}

		CAN2_0x2ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x2ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x20A):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x2ff_Tx_Data[2] << 8 | CAN2_0x2ff_Tx_Data[3]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}

		CAN2_0x2ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x2ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	case (0x20B):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN2_0x2ff_Tx_Data[4] << 8 | CAN2_0x2ff_Tx_Data[5]);
			motor->i_send += (int16_t)PID_Model1_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control2)
		{
			motor->i_send = (int16_t)PID_Model2_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		if (model == pid_control3)
		{
			motor->i_send = (int16_t)PID_Model3_Update(&motor->v_pid_object, (float)motor->omega, target);
		}
		CAN2_0x2ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN2_0x2ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
			break;
	}

	}
}
}

//电机位置环控制
//target是相对于电机零点的位置，单位为圈数
void DJmotor_location_change(DJ_Motors_t *motor, enum pid_control model, float target,float real)
{

	switch (motor->can_id)
	{
			case (0x201):
			case (0x202):
			case (0x203):
			case (0x204):
			case (0x209):
			{
				if (model == pid_control1)
				{
					motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, real, target);
				}
				if (model == pid_control2)
				{
					motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, real, target);
				}
				if (model == pid_control3)
				{
					motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, real, target);
				}
					break;
			}

			case (0x205):
			case (0x206):
			case (0x207):
			case (0x208):
			{
				if (model == pid_control1)
				{
					motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, real, target);
				}
				if (model == pid_control2)
				{
					motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, real, target);
				}
				if (model == pid_control3)
				{
					motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, real, target);
				}
		//		if (model == pid_control4)
		//		{
		//			motor->target_v = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, target);
		//		}
					break;
			}
	}
}
