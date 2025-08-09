/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "BSP_Can.h"
#include "BSP_Usart.h"

#include "Cloud_Control.h"
#include "Omni_Chassis.h"
#include "Protocol_Judgement.h"

#include "DT7.h"
#include "Task_LED.h"
#include "BSP_Test.h"
#include "Saber_C3.h"
#include "MA600_use.h"
#include "N100.h"
#include "UI.h"
#include "PowerControl.h"
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

/***********Queues************/
QueueHandle_t CAN1_ReceiveHandle;
QueueHandle_t CAN2_ReceiveHandle;
QueueHandle_t CAN_SendHandle;
/***********Tasks*************/
osThreadId Task_Can1MsgRecHandle;
osThreadId Task_Can2MsgRecHandle;
osThreadId Task_CanSendHandle;
osThreadId Task_SPI2MsgRecHandle;
osThreadId Move_DataHandle;
osThreadId led_RGB_flow_handle;
osThreadId Robot_Control_Handle;
osThreadId Robot_UI_Handle;
osThreadId Task_OffLineCheck_Handle;
osThreadId Task_VofaAssistHandle;

/* USER CODE END Variables */
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Can1Receives(void const *argument);
extern void Can2Receives(void const *argument);
extern void AllCanSend(void const *argument);
extern void SPI2Receives(void const *argument);
extern void Robot_Control(void const *argument);
extern void Show_Data(void const *argument);
extern void Robot_UI(void const *argument);
extern void Off_Line_Check(void const *argument);
extern void Vofa_Assist(void const *argument);


/* USER CODE END FunctionPrototypes */

void ALL_Init(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* add queues, ... */
	 /* definition and creation of CAN1_Receive */
    CAN1_ReceiveHandle = xQueueCreate(32, sizeof(Can_Export_Data_t));

    /* definition and creation of CAN2_Receive */
    CAN2_ReceiveHandle = xQueueCreate(64, sizeof(Can_Export_Data_t));

    /* definition and creation of CAN_Send */
    CAN_SendHandle = xQueueCreate(32, sizeof(Can_Send_Data_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartTask */
  osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
   /* definition and creation of Can1ReceiveTask */
    osThreadDef(Can1_ReceiveTask, Can1Receives, osPriorityRealtime, 0, 128);
    Task_Can1MsgRecHandle = osThreadCreate(osThread(Can1_ReceiveTask), NULL);

    /* definition and creation of Can1ReceiveTask */
    osThreadDef(Can2_ReceiveTask, Can2Receives, osPriorityRealtime, 4, 256);
    Task_Can2MsgRecHandle = osThreadCreate(osThread(Can2_ReceiveTask), NULL);
	
    /* definition and creation of CanSendTask */
    osThreadDef(Can_SendTask, AllCanSend, osPriorityRealtime, 0, 256);
    Task_CanSendHandle = osThreadCreate(osThread(Can_SendTask), NULL);
	
	/* definition and creation of LED_RGB_Flow_Task */
    osThreadDef(led, led_RGB_flow_task, osPriorityAboveNormal, 0, 128);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
	
	/* definition and creation of Robot_Control_Task */
    osThreadDef(Robot_Control_Task, Robot_Control, osPriorityRealtime, 0, 256);
    Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);
	
	/* definition and creation of Robot_Control_Task */
    osThreadDef(Robot_UI_Task, Robot_UI, osPriorityHigh, 0, 128);
    Robot_UI_Handle = osThreadCreate(osThread(Robot_UI_Task), NULL);		
		
	/* definition and creation of Task_OffLineCheck_Handle */
    osThreadDef(Task_OffLineCheck_Handle, Off_Line_Check, osPriorityHigh, 0, 128);
    Task_OffLineCheck_Handle = osThreadCreate(osThread(Task_OffLineCheck_Handle), NULL);
		
	/* definitiom and creation of SPI2_Receive_Task */
	  osThreadDef(SPI2_ReceiveTask, SPI2Receives, osPriorityRealtime, 0, 256);
    Task_SPI2MsgRecHandle = osThreadCreate(osThread(SPI2_ReceiveTask), NULL);
		
	/* definition and creation of vofa_Task */
	  osThreadDef(VofaAssistTask, Vofa_Assist, osPriorityRealtime, 0, 128);
    Task_VofaAssistHandle = osThreadCreate(osThread(VofaAssistTask), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ALL_Init */
/**
  * @brief  Function implementing the StartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ALL_Init */
void ALL_Init(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN ALL_Init */
  /* Infinite loop */
	for(;;)
	{
		taskENTER_CRITICAL();                 //�����ٽ���
	    /* CAN�жϳ�ʼ�� */
        Can_Fun.CAN_IT_Init(&hcan1, Can1_Type);
        Can_Fun.CAN_IT_Init(&hcan2, Can2_Type);
		  /*编码器初始化*/
		  MA600sInit();
		  /*���̳�ʼ��*/
		   chassis_init();
		  /*ң������ʼ��*/
		   DT7_Init();
	    /* PID��ʼ�� */
//	    for(int i = 0; i <= 3; i++)
//        {
//            Incremental_PIDInit(&M3508_Array_Pid[i], 20.0f, 0.22f, 0, 16384, 6000);
//        }
		    /**Yaw轴电机PID初始化**/
        Position_PIDInit(&M6020s_YawIPID, 1550.0f, 0.1f, 1200.0, 0, 30000, 10000 , 6000);
        Position_PIDInit(&M6020s_YawOPID, 0.083f, 0.00001f, 0.0, 0, 30000, 10000 , 10000);
		    /**AimYaw电机PID初始化*/
        Position_PIDInit(&AutoAim_M6020s_YawIPID, 1700.0f, 60.f, 800.f, 500, 30000, 10000 , 10000);
        Position_PIDInit(&AutoAim_M6020s_YawOPID, 0.032f, 0.00001f, 0.05, 0, 30000, 10000 , 10000);
		/*�豸��ʼ��*/
        Cloud_Init();						  //云台初始化
		
		/* ��������ϵͳ */
		JudgeSystem_USART_Receive_DMA(&huart6);
		Saber_Init();							//Saber IMU初始化
									
		UI_FUN.UI_init();
		vTaskDelete(StartTaskHandle);         //删除启动任务
    taskEXIT_CRITICAL();                  //退出临界区	
	}
  /* USER CODE END ALL_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
