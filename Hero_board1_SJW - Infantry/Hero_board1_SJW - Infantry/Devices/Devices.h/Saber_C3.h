/**
 * @file Saber_C3.h
 * @author lxr(784457420@qq.com)
 * @brief 
 * @version 1.0
 * @date 2023-8-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 #ifndef __SABER_C3_H
 #define __SABER_C3_H
 
 #include "main.h"
 #include "usart.h"
 #include "M6020_Motor.h"
 #include "struct_typedef.h"
 typedef struct Saber_Angle_t
 {
	 fp32 RoLL;
	 fp32 Pitch;
	 fp32 Yaw;
	 fp32 X_Vel;
	 fp32 Y_Vel;
	 fp32 Z_Vel;
 }Saber_Angle_t;
 
/* ������DMA��ƴ�ӻ�������С,�Լ�֡β��λ�� */
#define Saber_Data_Length 76
#define Saber_Data_Buffer 152
#define Frame_End 75

extern uint8_t Saber_Rxbuffer[Saber_Data_Length];
extern uint8_t Saber_Montage[Saber_Data_Buffer];


	
extern Saber_Angle_t Saber_Angle;
  /**
  * @brief          IMU��ʼ�����񣬴򿪴���
  * @retval         none
  */
extern void Saber_Init(void);
 /**
  * @brief          IMUŷ���Ƕ�ȡ����
  * @retval         none
  */
extern void Saber_Read(void);
   /**
  * @brief          ֡ͷ֡β�ж�
  * @parameter[in]  ���������׵�ַ
  * @retval         ������Э�鷵��1�����򷵻�0��
  */
extern uint8_t Saber_Cheak(uint8_t *p);
 
#endif
