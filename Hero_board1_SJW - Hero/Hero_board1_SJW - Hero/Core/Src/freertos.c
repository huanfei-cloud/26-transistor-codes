/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "typedef.h"
#include "pid.h"
#include "BSP_Can.h"
#include "BSP_Usart.h"
#include "Cloud_Control.h"
#include "Shoot.h"
#include "Task_LED.h"
#include "SBUS.h"
#include "DT7.h"
#include "Saber_C3.h"
#include "Protocol_UpperComputer.h"
#include "Task_vofa.h"
#include "Task_J4310_onlineCheck.h"
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
QueueHandle_t CAN1_ReceiveHandle;              // can1接收队列
QueueHandle_t CAN2_ReceiveHandle;              // can2接收队列
QueueHandle_t CAN_SendHandle;                  // can发送队列
QueueHandle_t Communicate_ReceivefromPCHandle; // 从PC接收到的数据队列

/***********Tasks************/
osThreadId Task_Can1MsgRecHandle; // can1消息接收任务句柄
osThreadId Task_Can2MsgRecHandle; // can2消息接收任务句柄
osThreadId Task_CanSendHandle;    // can发送任务句柄
osThreadId Move_DataHandle;
osThreadId Robot_Control_Handle;          // 机器人控制任务句柄
osThreadId Task_CommunicateFromPC_Handle; // 从PC通信任务句柄
osThreadId Task_CommunicateToPC_Handle;   // 向PC通信任务句柄
osThreadId Task_DT7_Handle;               // 遥控器任务句柄;
osThreadId Task_VOFA_Handle;              // VOFA调试任务句柄;
osThreadId Task_J4310_onlineCheck_Handle; // J4310在线检查任务句柄;

/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Can1Receives(void const *argument);
extern void Can2Receives(void const *argument);
extern void AllCanSend(void const *argument);
extern void Robot_Control(void const *argument);
extern void USBCommunicateTask_Receive(void const *argument);
extern void USBCommunicateTask_Send(void const *argument);
extern void DT7_Control(void const *argument);
extern void VOFA_Handle(void const *argument);
extern void J4310_onlineCheck(void const *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void ALL_Init(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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
void MX_FREERTOS_Init(void)
{
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
  CAN1_ReceiveHandle = xQueueCreate(16, sizeof(Can_Export_Data_t));

  /* definition and creation of CAN2_Receive */
  CAN2_ReceiveHandle = xQueueCreate(16, sizeof(Can_Export_Data_t));

  /* definition and creation of CAN_Send */
  CAN_SendHandle = xQueueCreate(16, sizeof(Can_Send_Data_t));

  /* definition and creation of Communicate_PC */
  Communicate_ReceivefromPCHandle = xQueueCreate(16, UpperCom_MAX_BUF);
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
  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Can1_ReceiveTask, Can1Receives, osPriorityAboveNormal, 0, 128);
  Task_Can1MsgRecHandle = osThreadCreate(osThread(Can1_ReceiveTask), NULL);

  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Can2_ReceiveTask, Can2Receives, osPriorityAboveNormal, 0, 128);
  Task_Can2MsgRecHandle = osThreadCreate(osThread(Can2_ReceiveTask), NULL);

  /* definition and creation of CanSendTask */
  osThreadDef(Can_SendTask, AllCanSend, osPriorityHigh, 0, 256);
  Task_CanSendHandle = osThreadCreate(osThread(Can_SendTask), NULL);

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

  /* definition and creation of Task_VOFA_Handle */
  osThreadDef(Task_VOFA_Handle, VOFA_Handle, osPriorityAboveNormal, 0, 128);
  Task_VOFA_Handle = osThreadCreate(osThread(Task_VOFA_Handle), NULL);

  /* definition and creation of Task_J4310_onlineCheck_Handle */
  osThreadDef(Task_J4310_onlineCheck_Handle, J4310_onlineCheck, osPriorityAboveNormal, 0, 128);
  Task_J4310_onlineCheck_Handle = osThreadCreate(osThread(Task_J4310_onlineCheck_Handle), NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
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
void ALL_Init(void const *argument)
{
  /* USER CODE BEGIN ALL_Init */
  /* Infinite loop */
  for (;;)
  {
    taskENTER_CRITICAL();
    /*********初始化两个CAN控制协议，使用中断模式*********/
    Can_Fun.CAN_IT_Init(&hcan1, Can1_Type);
    Can_Fun.CAN_IT_Init(&hcan2, Can2_Type);
    /*********初始化PID*********/
    fuzzy_init(&fuzzy_pid_shoot_F1, 100, -100, 25, 0.1, 10);
    fuzzy_init(&fuzzy_pid_shoot_F2, 100, -100, 25, 0.1, 10);
    fuzzy_init(&fuzzy_pid_shoot_F3, 100, -100, 25, 0.1, 10);
    fuzzy_init(&fuzzy_pid_shoot_B1, 100, -100, 25, 0.1, 10);
    fuzzy_init(&fuzzy_pid_shoot_B2, 100, -100, 25, 0.1, 10);
    fuzzy_init(&fuzzy_pid_shoot_B3, 100, -100, 25, 0.1, 10);

    fuzzy_init(&fuzzy_pid_bullet_v, 30, -30, 5, 0.01, 3);
    fuzzy_init(&fuzzy_pid_bullet_l, 10, -10, 0.1, 0.01, 1);

    Incremental_PIDInit(&M3508_FricF1_Pid, 40.f, 0, 30, 20000, 5000);
    Incremental_PIDInit(&M3508_FricF2_Pid, 40.f, 0, 30, 20000, 5000);
    Incremental_PIDInit(&M3508_FricF3_Pid, 40.f, 0, 30, 20000, 5000);
    Incremental_PIDInit(&M3508_FricB1_Pid, 40.f, 0, 30, 20000, 5000);
    Incremental_PIDInit(&M3508_FricB2_Pid, 40.f, 0, 30, 20000, 5000);
    Incremental_PIDInit(&M3508_FricB3_Pid, 40.f, 0, 30, 20000, 5000);

    Position_PIDInit(&M3508_DialV_Pid, 0.4f, 0.015f, 0.3, 0.5, 2000, 1000, 500);
    Incremental_PIDInit(&M3508_DialI_Pid, 15.0f, 2.5f, 8, 25000, 10000);

    Cloud_Init();
#if (RemoteControlMethod == TDF)
    SBUS_Init(); // 天地飞遥控器
#elif (RemoteControlMethod == DT7)
    DT7_Init(); // 大疆DT7遥控器
#endif
    vTaskDelete(StartTaskHandle); // 删除启动任务
    taskEXIT_CRITICAL();          // 退出临界区
    osDelay(1);
  }
  /* USER CODE END ALL_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
