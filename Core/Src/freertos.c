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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "telecontrol.h"
#include "motor3508.h"
#include <stdbool.h>
#include <string.h>
#include "chassis.h"
#include "communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rc_task_frame[RC_FRAME_LEN];
volatile uint32_t rc_task_frame_count = 0;
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
/* Definitions for Control_Parsing */
osThreadId_t Control_ParsingHandle;
const osThreadAttr_t Control_Parsing_attributes = {
  .name = "Control_Parsing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for motor3508 */
osThreadId_t motor3508Handle;
const osThreadAttr_t motor3508_attributes = {
  .name = "motor3508",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRcTask(void *argument);
void motor3508_speed_control(void *argument);
void up_down_communication(void *argument);

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
  /* creation of Control_Parsing */
  Control_ParsingHandle = osThreadNew(StartRcTask, NULL, &Control_Parsing_attributes);

  /* creation of motor3508 */
  motor3508Handle = osThreadNew(motor3508_speed_control, NULL, &motor3508_attributes);

  /* creation of communication */
  communicationHandle = osThreadNew(up_down_communication, NULL, &communication_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRcTask */
/**
  * @brief  Function implementing the Control_Parsing thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRcTask */
void StartRcTask(void *argument)
{
  /* USER CODE BEGIN StartRcTask */
    uint32_t next_tick = osKernelGetTickCount();
    uint32_t received_ms;
    (void)argument;
  /* Infinite loop */
  for(;;)
  {
      if (RC_TakeFrame(rc_task_frame, &received_ms))
    {
        rc_task_frame_count++;

        if (RC_ParseFrame(rc_task_frame, &rc_ctrl))
        {
            RC_MarkValidFrame(received_ms);
        }

    }

      (void)RC_CheckOnline(HAL_GetTick());

        /* 每隔 15 tick执行一轮。 */
      next_tick += 15U;

      /* 如果本轮超时，重新建立时间基准，避免连续追赶。 */
      if (osDelayUntil(next_tick) != osOK)
      {
          next_tick = osKernelGetTickCount();
      }

  }
  /* USER CODE END StartRcTask */
}

/* USER CODE BEGIN Header_motor3508_speed_control */
/**
* @brief Function implementing the motor3508 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor3508_speed_control */
void motor3508_speed_control(void *argument)
{
  /* USER CODE BEGIN motor3508_speed_control */
  uint32_t next_tick;
  (void)argument;
  next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    if(RC_online_return()&&Motor3508_OnlineCheck()&&rc_ctrl.rc.s[0]==2){
    Chassis_MecanumInverse(rc_ctrl.rc.ch[3]*5,-rc_ctrl.rc.ch[2]*5,-rc_ctrl.rc.ch[0]*5);
    }else {
      Motor3508_Stop();
    }

    next_tick += 1U;
      if (osDelayUntil(next_tick) != osOK)
    {
        next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END motor3508_speed_control */
}

/* USER CODE BEGIN Header_up_down_communication */
/**
* @brief Function implementing the communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_up_down_communication */
void up_down_communication(void *argument)
{
  /* USER CODE BEGIN up_down_communication */
  uint32_t next_tick;
  uint32_t frame_count_snapshot;
  uint8_t raw_frame[RC_FRAME_LEN];
  uint8_t tx_d1[COMMUNICATION_FRAME_SIZE];
  uint8_t tx_d2[COMMUNICATION_FRAME_SIZE];
  uint8_t tx_d3[COMMUNICATION_FRAME_SIZE] = {0};

  (void)argument;
  next_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    /* rc_task_frame 由遥控解析任务更新。复制时禁止任务切换，防止
     * 18 字节复制到一半时被新帧覆盖，导致三条 CAN 报文不属于同一帧。
     */
    taskENTER_CRITICAL();
    frame_count_snapshot = rc_task_frame_count;
    if (frame_count_snapshot > 0U)
    {
      memcpy(raw_frame, rc_task_frame, RC_FRAME_LEN);
    }
    taskEXIT_CRITICAL();

    if (frame_count_snapshot > 0U && RC_online_return())
    {
      /* 经典 CAN 一帧最多 8 字节，把 DBUS 的 18 字节原始帧依次拆分：
       * D1 = byte 0~7，D2 = byte 8~15，D3 = byte 16~17 + 六字节 0。
       */
      memcpy(tx_d1, &raw_frame[0], COMMUNICATION_FRAME_SIZE);
      memcpy(tx_d2, &raw_frame[8], COMMUNICATION_FRAME_SIZE);
      memset(tx_d3, 0, sizeof(tx_d3));
      memcpy(tx_d3, &raw_frame[16], RC_FRAME_LEN - 16U);

      (void)Communication_Send(COMMUNICATION_TX_ID_D1, tx_d1);
      (void)Communication_Send(COMMUNICATION_TX_ID_D2, tx_d2);
      (void)Communication_Send(COMMUNICATION_TX_ID_D3, tx_d3);
    }

    next_tick += 1U;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END up_down_communication */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

