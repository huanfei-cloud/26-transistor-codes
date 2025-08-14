/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "BSP_Fdcan.h"
#include "M2006_Motor.h"
#include "BSP_Usart.h"
#include "Protocol_Judgement.h"
#include "UI.h"

#include "Cloud_Control.h"
#include "Shoot.h"
#include "Mecanum_Chassis.h"

#include "SBUS.h"
#include "DT7.h"
#include "BSP_Test.h"
#include "Saber_C3.h"
#include "Protocol_UpperComputer.h"
#include "Devices_Online_Check.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//---------------------------------------********Board1********---------------------------------------
#if (USING_BOARD == BOARD1)
    /***********Queues************/
    QueueHandle_t FDCAN1_ReceiveHandle;
    QueueHandle_t FDCAN2_ReceiveHandle;
    QueueHandle_t FDCAN_SendHandle;
    QueueHandle_t Judge_ReceiveHandle;
    QueueHandle_t Communicate_ReceivefromPCHandle;
    /***********Tasks*************/
    osThreadId Task_Fdcan1MsgRecHandle;
    osThreadId Task_Fdcan2MsgRecHandle;
    osThreadId Task_FdcanSendHandle;
    osThreadId Move_DataHandle;
    osThreadId Robot_Control_Handle;
	  osThreadId Task_CommunicateFromPC_Handle;
	  osThreadId Task_CommunicateToPC_Handle; //��λ��ͨѶ
    osThreadId Task_Sampling_Handle;        //IMU ң���� ����
    osThreadId Task_Online_Check_Handle;    //���߼��

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
    extern void Fdcan1Receives(void const *argument);
    extern void Fdcan2Receives(void const *argument);
    extern void AllFdcanSend(void const *argument);
    extern void Robot_Control(void const *argument);
    extern void Show_Data(void const *argument);
    extern void Robot_Control(void const *argument);
    extern void USBCommunicateTask_Receive(void const *argument);
	  extern void USBCommunicateTask_Send(void const *argument);
    extern void Fixed_Sampling(void const *argument);
    extern void Online_Check(void const *argument);
		
#elif (USING_BOARD == BOARD2)
    /***********Queues************/
    QueueHandle_t FDCAN1_ReceiveHandle;
    QueueHandle_t FDCAN2_ReceiveHandle;
    QueueHandle_t FDCAN_SendHandle;
    QueueHandle_t Judge_ReceiveHandle;
    QueueHandle_t Communicate_ReceivefromPCHandle;
    /***********Tasks*************/
    osThreadId Task_Fdcan1MsgRecHandle;//FDCAN1����
    osThreadId Task_Fdcan2MsgRecHandle;//FDCAN2����
    osThreadId Task_FdcanSendHandle;//FDCAN����
    osThreadId Robot_Control_Handle;//�����˿���
    osThreadId Task_CommunicateWithPC_Handle;//��λ��ͨѶ
    osThreadId Task_Sampling_Handle;//IMU ң���� ����
    osThreadId Task_Online_Check_Handle;//���߼��
		osThreadId Robot_UI_Handle;//UI
    /* USER CODE END Variables */
    osThreadId StartTaskHandle;

    /* Private function prototypes -----------------------------------------------*/
    /* USER CODE BEGIN FunctionPrototypes */
    extern void Fdcan1Receives(void const *argument);
    extern void Fdcan2Receives(void const *argument);
    extern void AllFdcanSend(void const *argument);
    extern void Robot_Control(void const *argument);
    extern void Fixed_Sampling(void const *argument);
    extern void Online_Check(void const *argument);
		extern void Robot_UI(void const *argument);

#endif

//�±�Ϊ���еĶ��к�����
///***********Queues************/
//QueueHandle_t FDCAN1_ReceiveHandle;
//QueueHandle_t FDCAN2_ReceiveHandle;
//QueueHandle_t FDCAN_SendHandle;
//QueueHandle_t Judge_ReceiveHandle;
///***********Tasks*************/
//osThreadId Task_Fdcan1MsgRecHandle;
//osThreadId Task_Fdcan2MsgRecHandle;
//osThreadId Task_FdcanSendHandle;
//osThreadId imuTaskHandle;
//osThreadId Judgement_handle;
//osThreadId Robot_Control_Handle;
///* USER CODE END Variables */


///* Private function prototypes -----------------------------------------------*/
///* USER CODE BEGIN FunctionPrototypes */
//extern void Fdcan1Receives(void const *argument);
//extern void Fdcan2Receives(void const *argument);
//extern void AllFdcanSend(void const *argument);
//extern void Robot_Control(void const *argument);
//extern void Show_Data(void const *argument);
//extern void Judgement_Handle(void const *argument);
//extern void Robot_Control(void const *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ALL_Init(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
#if (USING_BOARD == BOARD1)
    /* add queues, ... */
    /* definition and creation of CAN1_Receive */
    /* definition and creation of CAN1_Receive */
    FDCAN1_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

    /* definition and creation of CAN2_Receive */
    FDCAN2_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

    /* definition and creation of CAN_Send */
    FDCAN_SendHandle = xQueueCreate(16, sizeof(Fdcan_Send_Data_t));

    /* definition and creation of Communicate_PC */
    Communicate_ReceivefromPCHandle = xQueueCreate(16, UpperCom_MAX_BUF);

    /* definition and creation of Judge Receive */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* definition and creation of Fdcan1ReceiveTask */

    osThreadDef(Fdcan1_ReceiveTask, Fdcan1Receives, osPriorityAboveNormal, 0, 256);
    Task_Fdcan1MsgRecHandle = osThreadCreate(osThread(Fdcan1_ReceiveTask), NULL);

    /* definition and creation of Fdcan2ReceiveTask */
    osThreadDef(Fdcan2_ReceiveTask, Fdcan2Receives, osPriorityAboveNormal, 0, 256);
    Task_Fdcan2MsgRecHandle = osThreadCreate(osThread(Fdcan2_ReceiveTask), NULL);

    /* definition and creation of CanSendTask */
    osThreadDef(Fdcan_SendTask, AllFdcanSend, osPriorityHigh, 0, 256);
    Task_FdcanSendHandle = osThreadCreate(osThread(Fdcan_SendTask), NULL);

    /* definition and creation of Robot_Control_Task */
    osThreadDef(Robot_Control_Task, Robot_Control, osPriorityHigh, 0, 256);
    Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);

    /* definition and creation of Task_CommunicateFromPC_Handle */
    osThreadDef(Task_CommunicateFromPC_Handle, USBCommunicateTask_Receive, osPriorityAboveNormal, 0, 256);
    Task_CommunicateFromPC_Handle = osThreadCreate(osThread(Task_CommunicateFromPC_Handle), NULL);

    /* definition and creation of Task_CommunicateToPC_Handle */
    osThreadDef(Task_CommunicateToPC_Handle, USBCommunicateTask_Send, osPriorityAboveNormal, 0, 256);
    Task_CommunicateToPC_Handle = osThreadCreate(osThread(Task_CommunicateToPC_Handle), NULL);

    /* definition and creation of Task_Sampling */
    osThreadDef(Task_Sampling, Fixed_Sampling, osPriorityAboveNormal, 0, 128);
    Task_Sampling_Handle = osThreadCreate(osThread(Task_Sampling), NULL);

   /* definition and creation of Task_Monitor */
//    osThreadDef(Task_Online_Check, Online_Check, osPriorityAboveNormal, 0, 128);
//    Task_Online_Check_Handle = osThreadCreate(osThread(Task_Online_Check), NULL);

#elif (USING_BOARD == BOARD2)
    /* add queues, ... */
    /* definition and creation of CAN1_Receive */
    FDCAN1_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

    /* definition and creation of CAN2_Receive */
    FDCAN2_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

    /* definition and creation of CAN_Send */
    FDCAN_SendHandle = xQueueCreate(16, sizeof(Fdcan_Send_Data_t));


    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of StartTask */
    osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
    StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* definition and creation of Fdcan1ReceiveTask */
    osThreadDef(Fdcan1_ReceiveTask, Fdcan1Receives, osPriorityAboveNormal, 0, 256);
    Task_Fdcan1MsgRecHandle = osThreadCreate(osThread(Fdcan1_ReceiveTask), NULL);

    /* definition and creation of Fdcan2ReceiveTask */
    osThreadDef(Fdcan2_ReceiveTask, Fdcan2Receives, osPriorityAboveNormal, 0, 256);
    Task_Fdcan2MsgRecHandle = osThreadCreate(osThread(Fdcan2_ReceiveTask), NULL);

    /* definition and creation of FdcanSendTask */
    osThreadDef(Fdcan_SendTask, AllFdcanSend, osPriorityNormal, 0, 256);
    Task_FdcanSendHandle = osThreadCreate(osThread(Fdcan_SendTask), NULL);

    /* definition and creation of Robot_Control_Task */
    osThreadDef(Robot_Control_Task, Robot_Control, osPriorityHigh, 0, 256);
    Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);

    /* definition and creation of Task_Sampling */
    osThreadDef(Task_Sampling, Fixed_Sampling, osPriorityHigh, 0, 128);
    Task_Sampling_Handle = osThreadCreate(osThread(Task_Sampling), NULL);

    /* definition and creation of Task_Monitor */
    osThreadDef(Task_Online_Check, Online_Check, osPriorityAboveNormal, 0, 128);
    Task_Online_Check_Handle = osThreadCreate(osThread(Task_Online_Check), NULL);

		/* definition and creation of Robot_UI_Task */
    osThreadDef(Robot_UI_Task, Robot_UI, osPriorityHigh, 0, 256);
    Robot_UI_Handle = osThreadCreate(osThread(Robot_UI_Task), NULL);

#endif
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ALL_Init */
/**
* @brief Function implementing the StartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ALL_Init */
void ALL_Init(void const * argument)
{
  /* USER CODE BEGIN ALL_Init */
    /* Infinite loop */
    for (;;)
    {
        taskENTER_CRITICAL();                 //�����ٽ���
        /* FDCAN�жϳ�ʼ�� */
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan1, Fdcan1_Type);
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan2, Fdcan2_Type);

#if (USING_BOARD == BOARD1)
				/*ģ��PID��ʼ��*/
				fuzzy_init(&fuzzy_pid_fric_l,100,-1500,3,0.005,10);
				fuzzy_init(&fuzzy_pid_fric_r,100,-1500,3,0.005,10);
				fuzzy_init(&fuzzy_pid_fric_u,100,-1500,3,0.005,10);
			  fuzzy_init(&fuzzy_pid_dial_l,10,-10,3,0.01,3);
				fuzzy_init(&M6020s_YawO_FuzzyPID,10,-10,0,0,0);
				fuzzy_init(&M6020s_YawI_FuzzyPID,10,-10,0,0,0);
				fuzzy_init(&M6020s_AimYawO_FuzzyPID,10,-10,0,0,0);
				fuzzy_init(&M6020s_AimYawI_FuzzyPID,10,-10,0,0,0);
			
        //Ħ����PID����
        PID_Model4_Update_PIDInit(&M3508_FricL_Pid, 60.0f, 0.012f, 63, 40,0, 25000, 10000, 10000);//��Ħ����
        PID_Model4_Update_PIDInit(&M3508_FricR_Pid, 60.0f, 0.012f, 63, 40,0, 25000, 10000, 10000);//��Ħ����
			  PID_Model4_Update_PIDInit(&M3508_FricU_Pid, 60.0f, 0.012f, 63, 40,0, 25000, 10000, 10000);//��Ħ����
        //���̵��PID����
        Position_PIDInit(&M3508_DialV_Pid, 0.4f, 0.015f, 0.3,0, 2000, 1000, 500);//λ�û�
        Incremental_PIDInit(&M3508_DialI_Pid, 15.f, 2.5f, 8, 25000, 10000);;//�ٶȻ�

        /**Yaw��Ƕ����⻷**/
//        PID_Model4_Update_PIDInit(&M6020s_YawIPID, 1600.0f, 0.1f, 1200.0, 800, 30000, 10000, 5000);
//        PID_Model4_Update_PIDInit(&M6020s_YawOPID, 0.086,0,0.1,0.1, 30000, 10000, 10000);    //0.086,0,0.1
        PID_Model4_Update_PIDInit(&M6020s_YawIPID, 1600.0f, 0.1f, 1200.0, 800,10, 30000, 10000, 5000);
        PID_Model4_Update_PIDInit(&M6020s_YawOPID, 0.1,0,0.1,0.1,10, 30000, 10000, 10000);    //0.086,0,0.1
        PID_Model4_Update_PIDInit(&M6020s_AimYawIPID, 2000.0f, 50.f,0, 1500.f, 300.f, 25000, 15000, 15000);
        PID_Model4_Update_PIDInit(&M6020s_AimYawOPID, 0.02, 0.f, 0.02,0.f,0, 30000, 5000, 5000);
				
				Position_PIDInit(&Chassis_Follow_Pid,10.0,0.1f,0.0f,0,660,100,100);

			  J4310_Fun.J4310_Enable();
				//J4310_Fun.J4310_Save_Pos_Zero();  //������λ
		    //DevicesMonitor_FUN.DevicesInit();
        JudgeSystem_FUN.JudgeSystem_USART_Receive_DMA(&huart1);
        //ң������ʼ��
        DT7_Fun.DT7_Init();//DT7��

		//����Ϊ���Դ��� ��ʽ������ɾ��
		ControlMes.shooter_42mm_heat_now=0;
		ControlMes.shooter_42mm_heat_limit = 200;

#elif (USING_BOARD == BOARD2)
        /* PID��ʼ�� */
        /**Yaw��Ƕ����⻷**/
        //Position_PIDInit(&M6020s_YawIPID, 250.0f, 1.2f, 200,1, 30000, 10000, 10000);
        //Position_PIDInit(&M6020s_YawOPID, 0.18f, 0.0004f, 1.5,1, 30000, 10000, 10000);

        //���̵��PID����
        for (int i = 0; i <= 3; i++)
        {
            Incremental_PIDInit(&M3508_Chassis_Pid[i], 20.0f, 0.22f, 0, 14000, 3000);
        }

		//DevicesMonitor_FUN.DevicesInit();
        //����6��ʼ����������ϵͳ���մ��ڳ�ʼ��
      JudgeSystem_FUN.JudgeSystem_USART_Receive_DMA(&huart1);
		//UI��ʼ��
			UI_FUN.UI_init();
        //IMU��ʼ��
        Saber_Fun.Saber_Init();
#endif
        vTaskDelete(StartTaskHandle);         //ɾ����ǰ����
        taskEXIT_CRITICAL();                  //�˳��ٽ���
    }
  /* USER CODE END ALL_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
