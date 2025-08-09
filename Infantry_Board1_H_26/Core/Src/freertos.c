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
#include "Task_OffLineCheck.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "BSP_Fdcan.h"
#include "BSP_Usart.h"

#include "Cloud_Control.h"
#include "Shoot.h"

#include "Task_LED.h"
#include "SBUS.h"
#include "DT7.h"
#include "BSP_Test.h"
#include "Saber_C3.h"
#include "Protocol_UpperComputer.h"

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
QueueHandle_t FDCAN1_ReceiveHandle;// FDCAN1接收队列
QueueHandle_t FDCAN2_ReceiveHandle;// FDCAN2接收队列  
QueueHandle_t FDCAN_SendHandle;// FDCAN发送队列
QueueHandle_t Communicate_ReceivefromPCHandle;// PC通信接收队列
/***********Tasks*************/
osThreadId Task_Fdcan1MsgRecHandle;// CAN1消息接收任务句柄
osThreadId Task_Fdcan2MsgRecHandle;// CAN2消息接收任务句柄
osThreadId Task_FdcanSendHandle;// CAN发送任务句柄  
osThreadId Move_DataHandle;
osThreadId Robot_Control_Handle;// 机器人控制任务句柄  
osThreadId Task_CommunicateFromPC_Handle;//从PC通信的任务的句柄
osThreadId Task_CommunicateToPC_Handle;//向PC通信的任务的句柄
osThreadId Task_OffLineCheck_Handle;//离线检查的任务的句柄
osThreadId Task_DT7_Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Fdcan1Receives(void const *argument);
extern void Fdcan2Receives(void const *argument);
extern void AllFdcanSend(void const *argument);
extern void Robot_Control(void const *argument);
extern void USBCommunicateTask_Receive(void const *argument);
extern void USBCommunicateTask_Send(void const *argument);
extern void Off_Line_Check(void const *argument);
extern void DT7_Control(void const *argument);

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
  /* add queues, ... */
	
  /* definition and creation of CAN1_Receive */
  FDCAN1_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

  /* definition and creation of CAN2_Receive */
  FDCAN2_ReceiveHandle = xQueueCreate(16, sizeof(Fdcan_Export_Data_t));

  /* definition and creation of CAN_Send */
  FDCAN_SendHandle = xQueueCreate(16, sizeof(Fdcan_Send_Data_t));

  /* definition and creation of Communicate_PC */
  Communicate_ReceivefromPCHandle = xQueueCreate(16, UpperCom_MAX_BUF);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */   
  //osThreadDef(线程名, 函数名, 优先级, 实例数, 栈大小);线程句柄 = osThreadCreate(osThread(线程名), NULL);
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, ALL_Init, osPriorityAboveNormal, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(OfflineCheckTask, Off_Line_Check, osPriorityAboveNormal, 0, 128);
  Task_OffLineCheck_Handle = osThreadCreate(osThread(OfflineCheckTask), NULL);
  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Fdcan1_ReceiveTask, Fdcan1Receives, osPriorityAboveNormal, 0, 128);
  Task_Fdcan1MsgRecHandle = osThreadCreate(osThread(Fdcan1_ReceiveTask), NULL);

  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Fdcan2_ReceiveTask, Fdcan2Receives, osPriorityAboveNormal, 0, 128);
  Task_Fdcan2MsgRecHandle = osThreadCreate(osThread(Fdcan2_ReceiveTask), NULL);

  /* definition and creation of CanSendTask */
  osThreadDef(Fdcan_SendTask, AllFdcanSend, osPriorityHigh, 0, 256);
  Task_FdcanSendHandle = osThreadCreate(osThread(Fdcan_SendTask), NULL);

  /* definition and creation of Robot_Control_Task */
  osThreadDef(Robot_Control_Task, Robot_Control, osPriorityHigh, 0, 256);
  Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);

  /* definition and creation of Task_CommunicateToPC_Handle */
 osThreadDef(Task_CommunicateToPC_Handle, USBCommunicateTask_Send, osPriorityAboveNormal, 0, 128);
  Task_CommunicateToPC_Handle = osThreadCreate(osThread(Task_CommunicateToPC_Handle), NULL);
	
  /* definition and creation of Task_CommunicateFromPC_Handle */
  osThreadDef(Task_CommunicateFromPC_Handle, USBCommunicateTask_Receive, osPriorityAboveNormal, 0, 128);
  Task_CommunicateFromPC_Handle = osThreadCreate(osThread(Task_CommunicateFromPC_Handle), NULL);

  /* definition and creation of Task_DT7_Handle */
  osThreadDef(Task_DT7_Handle, DT7_Control, osPriorityHigh, 0, 256);
  Task_DT7_Handle = osThreadCreate(osThread(Task_DT7_Handle), NULL);

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
        taskENTER_CRITICAL();        
        //包含在一个无限循环for(;;)中，但实际只会执行一次，因为最后会删除任务
        /* 初始化两个FDCAN控制器（CAN FD协议），使用中断模式 */
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan1, Fdcan1_Type);
        Fdcan_Fun.FDCAN_IT_Init(&hfdcan2, Fdcan2_Type);
			  /*初始化4个模糊PID控制器：*/
				fuzzy_init(&fuzzy_pid_shoot_l,100,-100,25,0.1,10);//左侧射击控制
				fuzzy_init(&fuzzy_pid_shoot_r,100,-100,25,0.1,10);//右侧射击控制
				fuzzy_init(&fuzzy_pid_bullet_v,30,-30,5,0.01,3);//子弹速度控制	
				fuzzy_init(&fuzzy_pid_bullet_l,10,-10,0.1,0.01,1);//子弹位置控制	

        /*增量式PID初始化*/
        Incremental_PIDInit(&M3508_FricL_Pid, 40.f, 0, 30, 20000, 5000);//左摩擦轮
        Incremental_PIDInit(&M3508_FricR_Pid, 40.f, 0, 30, 20000, 5000);//右摩擦轮
        Position_PIDInit(&M2006_DialV_Pid, 20.0f, 0.01f, 0.5, 0.5, 20000, 8000, 700);//拨弹电机weizhihuan 20.0 0.01 0.5 0.5
        Incremental_PIDInit(&M2006_DialI_Pid, 12.5f, 0.5f, 3, 10000, 1000);//拨弹电机速度环 12.5 0.5 3

				/*4310init*/
				J4310_Fun.J4310_Enable();
        Cloud_Init();
#if (RemoteControlMethod == TDF)
        SBUS_Init();//泰斗遥控器协议
#elif (RemoteControlMethod == DT7)
        DT7_Init();//大疆DT7遥控器 
#endif
        vTaskDelete(StartTaskHandle);         //删除启动任务
        taskEXIT_CRITICAL();                  //退出临界区
    }
  /* USER CODE END ALL_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
