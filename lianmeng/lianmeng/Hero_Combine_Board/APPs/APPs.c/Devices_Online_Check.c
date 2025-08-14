/**
 * @file Devices_Online_Check.c
 * @author Lxr
 * @brief
 * @version 1.0
 * @date 2024-01-24
 *
 * @copyright Copyright (c) 2024
 *
 */
 #include "Devices_Online_Check.h"
 
 #include "M3508_Motor.h"
 #include "DT7.h"
 #include "Saber_C3.h"
 
 const int Buzzer_DefaultPWM = 3000; //������������������3000���
 /* ģ�����߼��˼·��
	  ����ģ�ң�������� == �����˽���ʧ��ģʽ��
	  �����������豸���� == ���ù���ģ�����ʧ�ܻ��л���
	  �磺����ʶ�������ǡ�����ϵͳ��
  */
  /************************************************************************/
void DevicesMonitor_update(void);
//void DevicesMonitor_Init(void);
//void Buzzer_On(bool on, int volume);
//void DevicesMonitor_Alert(void);
void DevicesInit(void);
DevicesMonitor_FUN_t DevicesMonitor_FUN = DevicesMonitor_FUNGroundInit;
#undef DevicesMonitor_FUNGroundInit
Check_Fun_t Check_Fun[CHECK_LIST_Device] = {NULL};

static uint32_t
FPS_Calculate(uint16_t deltaTime)
{                                                  //FPS���㺯����
    return (1.0f / (double)(deltaTime)) * 1000.0f; // ��������ת��Ϊ��������������о��ȶ�ʧ
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) //��ȡ��ǰϵͳʱ�ӽ��Ĳ����FPS
{
    time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    *FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
    time->Last_WorldTime = time->WorldTime;
}


void DevicesInit(void)
{
    void *Check_Funnames[CHECK_LIST_Device] = {
		#if(USING_BOARD == BOARD1)
		M3508_FUN.Check_WheelM3508,
        DT7_Fun.Online_Check_DR16,
		M6020_Fun.Check_M6020,
		#else
		M3508_FUN.Check_WheelM3508,
		Saber_Fun.Saber_Check,	
        M6020_Fun.Check_M6020,
		#endif
    // �������ݻ�δ���   SuperCapacitorFUN.Check_SuperCapacitor,

   //2006���δ���     M2006_FUN.Check_M2006,
   //3508Ħ����δ���     M3508_FUN.Check_FrictionM3508,
   //Control_Vision_FUN.Check_Vision,
   //CANδ���     Can_Fun.Check_CAN,
   //����ϵͳδ���     JudgeSystem_FUN.Check_Judge,
    };
    for (uint8_t i = 0; i < CHECK_LIST_Device; i++)
    {
        Check_Fun[i].GetDevices = (void (*)(void))Check_Funnames[i];
    }
}


/**
  * @brief  ����豸֡�����㣬ÿ200ms����һ��
  * @param  None
  * @retval None
  */
void DevicesMonitor_update(void)
{
    /********************��⿪ʼ********************/
    for (uint8_t i = 0; i < CHECK_LIST_Device; i++)
    {
        Check_Fun[i].GetDevices();
    }

//    /*****************�����־λ���*****************/
//    if (imu_Export.OffLineFlag)
//    {
//        ReversefollowFlag.NewFollowFlag = 1;
//    }

//    if (DR16_Export_Data.OffLineFlag)
//    {
//        //ȫ��ʧ��
//        Robot_control_FUN.Robot_Disable();
//    }
//    else
//    {
//        Robot_control_FUN.Robot_Enable();
//    }
}


/**
 * @brief ��������ʼ��
 * 
 * @return  
 */
//void DevicesMonitor_Init(void)
//{

//    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//}

///**
// * @brief ���Ʒ������Ŀ��ؽӿڡ�
// * 
// * @param on 
// * @return  
// */
//void Buzzer_On(bool on, int volume)
//{
//    int pwm = volume;
//    if (on == 0)
//    {
//        pwm = 0;
//    }

//    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm); 
//}

/**
 * @brief ��������
 * 
 * @return  
 */
//int WorkStatus_AlertCount = 0;
//void DevicesMonitor_Alert(void)
//{
//    WorkStatus_AlertCount++;
//    Buzzer_On(false, Buzzer_DefaultPWM);

////    if (Robot.WorkStatus != WorkStatus_Normal)
////    {
////        switch (WorkStatus_AlertCount)
////        {
////        case 1:
////            Buzzer_On(true, Buzzer_DefaultPWM);
////            break;
////        case 2:
////            Buzzer_On(false, Buzzer_DefaultPWM);
////            break;
////        default:
////            WorkStatus_AlertCount = 0;
////        }
////    }
//}
