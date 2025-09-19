/**
 * @file bsp_motor.c
 * @author xyz
 * @brief motor相关处理
 * @version 1.1
 * @date 2024-09-18
 * @copyright Transistor BUAA
 */

#include "motor.h"

/**
 * @brief motor初始化
 * @param 电机处理结构体
 * @param id(默认电机1对应电调ID1，对应pid结构体1
 */
void motor_init(struct Struct_MOTOR_Manage_Object *motor, uint16_t _motor_id)
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
	motor->zero_flag = 0;
	motor->UpdateFlag = 0;
}

float encoder_to_quanshu(int32_t encoder)
{

	return ((float)encoder / 8192);
}

int32_t quanshu_to_encoder(float quanshu)
{
	return (int32_t)(quanshu * 8192);
}

/**
 * @brief 更新电机的encoder相关记录
 * @param *motor 要用pid控制的电机的地址
 */

// 由于电机上电时会有编码器值，而电机初始化时pre_encoder为0，所以上电时motor的total_encoder在-4096~4096之间
void motor_encoder_state_change(struct Struct_MOTOR_Manage_Object *motor)
{
	motor->now_encoder = motor->encoder;
	motor->delta_encoder = (int32_t)(motor->now_encoder - motor->pre_encoder);
	if (motor->zero_flag == 1) // 必须保证pre_encoder不为0才能判断圈数
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
	if (motor->zero_flag == 0)
	{
		motor->zero_encoder = motor->encoder;
		motor->zero_flag = 1;
	}
	motor->total_encoder = (motor->total_round) * 8192 + (int32_t)motor->now_encoder - motor->zero_encoder;
	motor->pre_encoder = motor->now_encoder;
}

void get_motor_data(struct Struct_MOTOR_Manage_Object *motor, uint8_t *data)
{
	motor->encoder = (uint16_t)data[0] << 8 | data[1];
	motor->omega = (uint16_t)data[2] << 8 | data[3];
	motor->torque = (uint16_t)data[4] << 8 | data[5];
	motor->temperaure = (uint16_t)data[6] << 8;
}

void motor_velocity_change(struct Struct_MOTOR_Manage_Object *motor, enum pid_control model, float target)
{
	switch (motor->can_id)
	{
	case (0x201):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x200_Tx_Data[0] << 8 | CAN_0x200_Tx_Data[1]);
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
		CAN_0x200_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x200_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x202):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x200_Tx_Data[2] << 8 | CAN_0x200_Tx_Data[3]);
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

		CAN_0x200_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x200_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x203):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x200_Tx_Data[4] << 8 | CAN_0x200_Tx_Data[5]);
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
		CAN_0x200_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x200_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x204):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x200_Tx_Data[6] << 8 | CAN_0x200_Tx_Data[7]);
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
		CAN_0x200_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x200_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x205):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x1ff_Tx_Data[0] << 8 | CAN_0x1ff_Tx_Data[1]);
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
		CAN_0x1ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x1ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x206):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x1ff_Tx_Data[2] << 8 | CAN_0x1ff_Tx_Data[3]);

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
		CAN_0x1ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x1ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x207):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x1ff_Tx_Data[4] << 8 | CAN_0x1ff_Tx_Data[5]);

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
		if (model == pid_control4)
		{
			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_bullet_v, (float)motor->omega, target);
		}
		CAN_0x1ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x1ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x208):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x1ff_Tx_Data[6] << 8 | CAN_0x1ff_Tx_Data[7]);
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

		CAN_0x1ff_Tx_Data[6] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x1ff_Tx_Data[7] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x209):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x2ff_Tx_Data[0] << 8 | CAN_0x2ff_Tx_Data[1]);
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
		if (model == pid_control4)
		{
			motor->i_send = (int16_t)PID_Model4_Update(&motor->v_pid_object, &fuzzy_pid_pitch_v, (float)motor->omega, target);
		}

		CAN_0x2ff_Tx_Data[0] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x2ff_Tx_Data[1] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x20A):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x2ff_Tx_Data[2] << 8 | CAN_0x2ff_Tx_Data[3]);
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

		CAN_0x2ff_Tx_Data[2] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x2ff_Tx_Data[3] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	case (0x20B):
	{
		if (model == pid_control1)
		{
			motor->i_send = (int16_t)(CAN_0x2ff_Tx_Data[4] << 8 | CAN_0x2ff_Tx_Data[5]);
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
		CAN_0x2ff_Tx_Data[4] = ((uint16_t)motor->i_send) >> 8;
		CAN_0x2ff_Tx_Data[5] = ((uint16_t)motor->i_send) & 0xff;
		break;
	}
	
	}
}

// 电机位置环控制
// target是相对于电机零点的位置，单位为圈数
void motor_location_change(struct Struct_MOTOR_Manage_Object *motor, enum pid_control model, float target)
{

	switch (motor->can_id)
	{
	case (0x201):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x202):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x203):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x204):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x205):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control4)
		{
			motor->target_v = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x206):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control4)
		{
			motor->target_v = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x207):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control4)
		{
			motor->target_v = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	case (0x208):
	{
		if (model == pid_control1)
		{
			motor->target_v += (int16_t)PID_Model1_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control2)
		{
			motor->target_v = (int16_t)PID_Model2_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control3)
		{
			motor->target_v = (int16_t)PID_Model3_Update(&motor->l_pid_object, motor->total_encoder, quanshu_to_encoder(target));
		}
		if (model == pid_control4)
		{
			motor->target_v = (int16_t)PID_Model4_Update(&motor->l_pid_object, &fuzzy_pid_yaw_l, motor->total_encoder, quanshu_to_encoder(target));
		}
		break;
	}
	
	}
}
