/**
 * @file N100.h
 * @author lxr(784457420@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-11-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "N100.h"

uint8_t N100_Rxbuffer[56];
N100_AHRSData_Packet_t N100_Angle;
/*************
ʵ��16���Ƶ�can����ת���ɸ���������
****************/
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
	uint32_t transition_32;
	float tmp = 0;
	int sign = 0;
	int exponent = 0;
	float mantissa = 0;
	transition_32 = 0;
	transition_32 |= Data_4 << 24;
	transition_32 |= Data_3 << 16;
	transition_32 |= Data_2 << 8;
	transition_32 |= Data_1;
	sign = (transition_32 & 0x80000000) ? -1 : 1; // ����λ
	// �����Ʋ������ٰ�λ����㣬���������30��23λ��Ӧ��e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	// ��22~0ת��Ϊ10���ƣ��õ���Ӧ��xϵ��
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp = sign * mantissa * pow(2, exponent);
	return tmp;
}

long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4)
{
	uint32_t transition_32;
	transition_32 = 0;
	transition_32 |= Data_4 << 24;
	transition_32 |= Data_3 << 16;
	transition_32 |= Data_2 << 8;
	transition_32 |= Data_1;
	return transition_32;
}

/**
 * @brief          N100_IMU��ʼ�����񣬴򿪴���
 * @retval         none
 */
void N100_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, N100_Rxbuffer, sizeof(N100_Rxbuffer));
}

/**
 * @brief          N100_IMU���ݽ����������ṹ��
 * @retval         none
 */
void N100_Read(void)
{
	if (N100_Rxbuffer[1] == TYPE_AHRS && N100_Rxbuffer[2] == AHRS_LEN)
	{
		N100_Angle.RollSpeed = DATA_Trans(N100_Rxbuffer[7], N100_Rxbuffer[8], N100_Rxbuffer[9], N100_Rxbuffer[10]);		// ������ٶ�
		N100_Angle.PitchSpeed = DATA_Trans(N100_Rxbuffer[11], N100_Rxbuffer[12], N100_Rxbuffer[13], N100_Rxbuffer[14]); // �������ٶ�
		N100_Angle.YawSpeed = DATA_Trans(N100_Rxbuffer[15], N100_Rxbuffer[16], N100_Rxbuffer[17], N100_Rxbuffer[18]);	// ƫ�����ٶ�

		N100_Angle.Roll = DATA_Trans(N100_Rxbuffer[19], N100_Rxbuffer[20], N100_Rxbuffer[21], N100_Rxbuffer[22]);  // �����
		N100_Angle.Pitch = DATA_Trans(N100_Rxbuffer[23], N100_Rxbuffer[24], N100_Rxbuffer[25], N100_Rxbuffer[26]); // ������
		N100_Angle.Yaw = DATA_Trans(N100_Rxbuffer[27], N100_Rxbuffer[28], N100_Rxbuffer[29], N100_Rxbuffer[30]);   // ƫ����

		N100_Angle.Qw = DATA_Trans(N100_Rxbuffer[31], N100_Rxbuffer[32], N100_Rxbuffer[33], N100_Rxbuffer[34]); // ��Ԫ��
		N100_Angle.Qx = DATA_Trans(N100_Rxbuffer[35], N100_Rxbuffer[36], N100_Rxbuffer[37], N100_Rxbuffer[38]);
		N100_Angle.Qy = DATA_Trans(N100_Rxbuffer[39], N100_Rxbuffer[40], N100_Rxbuffer[41], N100_Rxbuffer[42]);
		N100_Angle.Qz = DATA_Trans(N100_Rxbuffer[43], N100_Rxbuffer[44], N100_Rxbuffer[45], N100_Rxbuffer[46]);
		N100_Angle.Timestamp = timestamp(N100_Rxbuffer[47], N100_Rxbuffer[48], N100_Rxbuffer[49], N100_Rxbuffer[50]); // ʱ���
	}
}
