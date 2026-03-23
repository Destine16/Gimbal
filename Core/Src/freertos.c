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
// 应用主任务: 5ms 周期运行,负责 robot_cmd / gimbal 两个应用模块调度
const osThreadAttr_t robotTask_attributes = {
  .name = "robotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for insTask */
osThreadId_t insTaskHandle;
// 姿态任务: 1ms 周期运行,负责 BMI088 采样结果处理和姿态解算
const osThreadAttr_t insTask_attributes = {
  .name = "insTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
// 电机控制任务: 1ms 周期运行,负责 GM6020 闭环控制和 CAN 输出
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for daemonTask */
osThreadId_t daemonTaskHandle;
// 守护任务: 低频运行,统一维护各模块在线计数和离线状态
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

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 调度器启动前完成一次性业务初始化; 任务本体只负责周期运行
  RobotInit();

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
  /* creation of robotTask */
  robotTaskHandle = osThreadNew(StartRobotTask, NULL, &robotTask_attributes);

  /* creation of insTask */
  insTaskHandle = osThreadNew(StartINSTask, NULL, &insTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartRobotTask */
  // robotTask 周期: 5ms
  // 职责: 运行应用层状态机,在 robot_cmd 和 gimbal 之间流转控制/反馈消息
  for(;;)
  {
    RobotTask();
    osDelay(5);
  }
  /* USER CODE END StartRobotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

