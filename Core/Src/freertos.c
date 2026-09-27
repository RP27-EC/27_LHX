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
#include "upper_tasks.h"
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
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for gimbalControl */
osThreadId_t gimbalControlHandle;
const osThreadAttr_t gimbalControl_attributes = {
  .name = "gimbalControl",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for shootControl */
osThreadId_t shootControlHandle;
const osThreadAttr_t shootControl_attributes = {
  .name = "shootControl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void up__down_communication(void *argument);
void gimbal_control(void *argument);
void shoot_control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of communication */
  communicationHandle = osThreadNew(up__down_communication, NULL, &communication_attributes);

  /* creation of gimbalControl */
  gimbalControlHandle = osThreadNew(gimbal_control, NULL, &gimbalControl_attributes);

  /* creation of shootControl */
  shootControlHandle = osThreadNew(shoot_control, NULL, &shootControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_up__down_communication */
/**
  * @brief  Function implementing the communication thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_up__down_communication */
void up__down_communication(void *argument)
{
  /* USER CODE BEGIN up__down_communication */
  (void)argument;
  UpperTasks_RunCommunication();
  /* USER CODE END up__down_communication */
}

/* USER CODE BEGIN Header_gimbal_control */
/**
* @brief 云台和升降控制任务。
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_control */
void gimbal_control(void *argument)
{
  /* USER CODE BEGIN gimbal_control */
  (void)argument;
  UpperTasks_RunControl();
  /* USER CODE END gimbal_control */
}

/* USER CODE BEGIN Header_shoot_control */
/**
* @brief 发射机构控制任务。
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_control */
void shoot_control(void *argument)
{
  /* USER CODE BEGIN shoot_control */
  (void)argument;
  UpperTasks_RunShoot();
  /* USER CODE END shoot_control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

