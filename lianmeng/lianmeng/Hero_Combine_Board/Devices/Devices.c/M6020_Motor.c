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

//ֱ��������Ӧ�ĵ���Ľṹ����������飬ֱ�۱��ں��ڵ��Թ۲�����ʹ�á�
M6020s_t M6020s_Yaw;                                    //IDΪ1
M6020s_t M6020s_Pitch;                                  //2
M6020s_t *M6020_Array[2] = {&M6020s_Yaw, &M6020s_Pitch}; //��Ӧ�����ID����Ϊ������+1
#define M6020_Amount 2
void M6020_setVoltage(int16_t uq1, int16_t uq2, int16_t uq3, int16_t uq4, uint8_t *data);
void M6020_getInfo(Fdcan_Export_Data_t RxMessage);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);
void Check_M6020(void);

M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit

/**
  * @brief  ����M6020�����ѹ��id��Ϊ1~4��
  * @param  uqx (x:1~4) ��Ӧid�ŵ���ĵ�ѹֵ����Χ -30000~0~30000
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
  * @brief  ��FDCAN�����л�ȡM6020�����Ϣ
  * @param  RxMessage 	FDCAN���Ľ��սṹ��
  * @retval None
  */

void M6020_getInfo(Fdcan_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.FDCAN_RxHeader.Identifier - M6020_READID_YAW; //��0��ʼ
    //������ݣ����ݸ�ʽ���C620���˵����P33
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
    //֡��ͳ�ƣ����ݸ��±�־λ
    M6020_Array[StdId]->InfoUpdateFrame++;
    M6020_Array[StdId]->InfoUpdateFlag = 1;

}

/*
*@brief  �趨M6020�����Ŀ��Ƕ�
* @param  motorName 	������� @ref M6623Name_e
*			angle		����ֵ����Χ 0~8191 ��������0��8191�ᵼ�µ���񵴣�Ҫ�����޷�
* @retval None
* */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/*************************************
* Method:    M6020_OverflowReset
* Returns:   void
* ˵�������˴˺����Խ��totalAngle ����������⡣
************************************/
void M6020_Reset(M6020s_t *m6020)
{
    //������ݣ����ݸ�ʽ���C620���˵����P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}

/**
 * @brief 6020���
 * 
 */
void Check_M6020(void)
{
	#if(USING_BOARD == BOARD2)
    //M6020���
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


