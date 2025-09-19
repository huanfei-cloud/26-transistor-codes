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
 
 typedef struct Saber_Angle_t
 {
	fp32 RoLL;
	fp32 Pitch;
	fp32 Yaw;
	 
	fp32 X_Vel;
	fp32 Y_Vel;
	fp32 Z_Vel;
	 
	fp32 X_Acc;
	fp32 Y_Acc;
	fp32 Z_Acc;
	 
	uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;
    uint32_t IMU_FPS;
	 
 }Saber_Angle_t;
 
 typedef struct
 {
	 void (*Saber_Init)(void);
	 void (*Saber_Read)(void);
	 uint8_t (*Saber_Check)(uint8_t *p);
	 void (*Saber_Online_Check)(void);
 }Saber_Fun_t;
 
 #define Saber_FunGroundInit 	 \
	{							 \
		&Saber_Init,			 \
		&Saber_Read,			 \
		&Saber_Check,			 \
		&Saber_Online_Check, 	 \
	}
 
/* 陀螺仪DMA和拼接缓冲区大小,以及帧尾的位置 */
#define Saber_Data_Length 76
#define Saber_Data_Buffer 152
#define Frame_End 75

extern uint8_t Saber_Rxbuffer[Saber_Data_Length];
extern uint8_t Saber_Montage[Saber_Data_Buffer];

extern Saber_Angle_t Saber_Angle;
extern Saber_Fun_t Saber_Fun;
 
#endif
