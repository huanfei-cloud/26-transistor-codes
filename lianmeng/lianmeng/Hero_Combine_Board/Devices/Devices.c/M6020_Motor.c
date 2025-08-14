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
#include "struct_typedef.h"

//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
M6020s_t M6020s_Yaw;                                    //ID为1
M6020s_t M6020s_Pitch;                                  //2
M6020s_t *M6020_Array[2] = {&M6020s_Yaw, &M6020s_Pitch}; //对应电机的ID必须为：索引+1
#define M6020_Amount 2
void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
void M6020_getInfo(Fdcan_Export_Data_t RxMessage);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);
void Check_M6020(void);

M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit

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

// 	// FDCAN_SendData(&hfdcan1, FDCAN_ID_STD, M6020_SENDID, data);
 }

/**
  * @brief  从FDCAN报文中获取M6020电机信息
  * @param  RxMessage 	FDCAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(Fdcan_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.FDCAN_RxHeader.Identifier - M6020_READID_YAW; //由0开始
    //解包数据，数据格式详见C620电调说明书P33
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
//	  if((M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle > 1000) || (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -1000) )  
//		{	
//				M6020_Array[StdId]->realAngle=M6020_Array[StdId]->lastAngle;
//		}
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.FDCANx_Export_RxMessage[6];

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

/*
*@brief  设定M6020电机的目标角度
* @param  motorName 	电机名字 @ref M6623Name_e
*			angle		电流值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
* @retval None
* */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/*************************************
* Method:    M6020_OverflowReset
* Returns:   void
* 说明：调运此函数以解决totalAngle 等溢出的问题。
************************************/
void M6020_Reset(M6020s_t *m6020)
{
    //解包数据，数据格式详见C620电调说明书P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}

/**
 * @brief 6020检测
 * 
 */
void Check_M6020(void)
{
	#if(USING_BOARD == BOARD2)
    //M6020检测
    for (int i = 0; i < 1; i++)
    {
        if (M6020_Array[i]->InfoUpdateFrame < 1)
        {
            M6020_Array[i]->OffLineFlag = 1;
        }
        else
        {
            M6020_Array[i]->OffLineFlag = 0;
        }
        M6020_Array[i]->InfoUpdateFrame = 0;
    }
	#else
	for (int i = 1; i < 2; i++)
    {
        if (M6020_Array[i]->InfoUpdateFrame < 1)
        {
            M6020_Array[i]->OffLineFlag = 1;
        }
        else
        {
            M6020_Array[i]->OffLineFlag = 0;
        }
        M6020_Array[i]->InfoUpdateFrame = 0;
    }
	#endif
}


