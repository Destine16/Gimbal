/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "robot.h"
#include "robot_def.h"

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

/* USER CODE END Variables */
/* Definitions for robotTask */
osThreadId_t robotTaskHandle;
const osThreadAttr_t robotTask_attributes = {
  .name = "robotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for insTask */
osThreadId_t insTaskHandle;
const osThreadAttr_t insTask_attributes = {
  .name = "insTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for daemonTask */
osThreadId_t daemonTaskHandle;
const osThreadAttr_t daemonTask_attributes = {
  .name = "daemonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRobotTask(void *argument);
extern void StartINSTask(void *argument);
extern void StartMotorTask(void *argument);
extern void StartDaemonTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 调度器启动前完成一次性业务初始化; 任务本体只负责周期运行
#if IMU_ONLY_BRINGUP_ENABLE
#else
  RobotInit();
#endif

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
#if !IMU_ONLY_BRINGUP_ENABLE
  /* creation of robotTask */
  robotTaskHandle = osThreadNew(StartRobotTask, NULL, &robotTask_attributes);
#endif

  /* creation of insTask */
  insTaskHandle = osThreadNew(StartINSTask, NULL, &insTask_attributes);

#if !IMU_ONLY_BRINGUP_ENABLE
  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
#endif

  /* creation of daemonTask */
  daemonTaskHandle = osThreadNew(StartDaemonTask, NULL, &daemonTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRobotTask */
/**
  * @brief  Function implementing the robotTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRobotTask */
void StartRobotTask(void *argument)
{
  /* USER CODE BEGIN StartRobotTask */
  // robotTask 周期: 1ms
  // 职责: 更新应用层状态机、扫描目标和 IMU 反馈到电机控制链路
  for(;;)
  {
    RobotTask();
    osDelay(1);
  }
  /* USER CODE END StartRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

