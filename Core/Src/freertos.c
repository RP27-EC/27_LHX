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
#include "communication.h"
#include "cloud_terrace.h"
#include "imu.h"
#include "parameter.h"
#include "motor3508.h"
#include "dial_motor.h"
#include "shoot_control.h"
#include "remote_state.h"
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
osThreadId_t communicationHandle; /* 板间通信与遥控解析任务句柄。 */
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle; /* 云台电机控制任务句柄。 */
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 516 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle; /* 发射机构控制任务句柄。 */
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void up__down_communication(void *argument);
void motor_control(void *argument);
void shoot(void *argument);

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

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(motor_control, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(shoot, NULL, &myTask03_attributes);

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
  uint32_t next_tick;
  Communication_RcControl_t remote_control;

  (void)argument;
  ShootControl_Init();
  RemoteState_Init();
  next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    Communication_Process();
    RemoteState_Update(&remote_control,
                       Communication_RC_Get(&remote_control));

    next_tick += 1U;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END up__down_communication */
}

/* USER CODE BEGIN Header_motor_control */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_control */
void motor_control(void *argument)
{
  /* USER CODE BEGIN motor_control */
  uint32_t next_tick;

  (void)argument;
  CloudTerrace_Init();
  next_tick = osKernelGetTickCount();
  for(;;)
  {
    /* 先更新 IMU，再用当次数据执行云台闭环。 */
    (void)GimbalImu_Update();
    CloudTerrace_Update();

    next_tick += CLOUD_CONTROL_PERIOD_TICKS;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END motor_control */
}

/* USER CODE BEGIN Header_shoot */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot */
void shoot(void *argument)
{
  /* USER CODE BEGIN shoot */
  uint32_t next_tick;

  (void)argument;
  next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    bool shoot_motors_online;
    RemoteState_t remote;
    RemoteShoot_t shoot_mode;

    /* 1 ms 更新在线状态，反馈超过 100 ms 未刷新即掉线。 */
    Motor3508_Heartbeat();
    DialMotor_Heartbeat();
    shoot_motors_online =
        Motor3508_OnlineCheck(SHOOT_LEFT_FRIC_MOTOR_ID) &&
        Motor3508_OnlineCheck(SHOOT_RIGHT_FRIC_MOTOR_ID) &&
        DialMotor_OnlineCheck();
    RemoteState_Get(&remote);

    /* 遥控拨杆沿用原单发/连发；键鼠左键短按单发、长按连发。 */
    shoot_mode = remote.online && remote.shoot_armed && shoot_motors_online ?
        remote.shoot : REMOTE_SHOOT_OFF;
    if (remote.keyboard_active)
    {
      ShootControl_UpdateKeyboard(shoot_mode,
                                  remote.shoot_single_request_count);
    }
    else
    {
      ShootControl_ResetKeyboard(remote.shoot_single_request_count);
      ShootControl_Update(shoot_mode, remote.right_up);
    }

    next_tick += SHOOT_CONTROL_PERIOD_TICKS;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END shoot */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

