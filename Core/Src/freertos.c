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
#include "parameter.h"
#include "imu.h"
#include "remote_state.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rc_task_frame[RC_FRAME_LEN]; /* 遥控解析任务使用的 DBUS 帧副本。 */
volatile uint32_t rc_task_frame_count = 0; /* 任务成功解析的遥控帧计数。 */
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
osThreadId_t Control_ParsingHandle; /* 遥控接收与解析任务句柄。 */
const osThreadAttr_t Control_Parsing_attributes = {
  .name = "Control_Parsing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for motor3508 */
osThreadId_t motor3508Handle; /* 底盘 3508 控制任务句柄。 */
const osThreadAttr_t motor3508_attributes = {
  .name = "motor3508",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for communication */
osThreadId_t communicationHandle; /* 上下板 CAN 通信任务句柄。 */
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
    RemoteState_Init();
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

      RemoteState_Update(&rc_ctrl, RC_CheckOnline(HAL_GetTick()));

        /* 按配置周期执行遥控解析和在线检测。 */
      next_tick += RC_TASK_PERIOD_TICKS;

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
  RemoteState_t remote;
  bool turn_hold;
  (void)argument;
  next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    /* BMI088 角速度与姿态保持 1 ms 更新，跟随模式直接使用 Z 轴角速度。 */
    (void)ChassisImu_Update();
    RemoteState_Get(&remote);
    turn_hold = false;
    if (!remote.online)
    { Chassis_TurnaroundReset(remote.turnaround_request_count); }
    else
    { turn_hold = Chassis_TurnaroundUpdate(remote.turnaround_request_count); }
    if (remote.online && Motor3508_OnlineCheck() && !turn_hold)
    {
      if (remote.mode == REMOTE_MODE_SPIN)
      {
        /* 左下档按云台朝向平移；仅右上档叠加底盘自旋。 */
        Chassis_FollowReset();
        Chassis_SpinUpdate(remote.channel[3] * CHASSIS_FORWARD_SCALE,
                           remote.channel[2] * CHASSIS_LEFT_SCALE,
                           remote.spin_enabled);
      }
      else if (remote.mode == REMOTE_MODE_FOLLOW)
      {
        Chassis_SpinReset();
        /* 上档由云台相对车头角度驱动旋转，左右摇杆只控制 Yaw。 */
        Chassis_FollowUpdate(remote.channel[3] * CHASSIS_FORWARD_SCALE,
                             remote.channel[2] * CHASSIS_LEFT_SCALE,
                             (float)remote.channel[0]);
      }
      else if (remote.mode == REMOTE_MODE_MECHANICAL)
      {
        /* 中档使用手动底盘控制，同时上板 Yaw 锁车头。 */
        Chassis_FollowReset();
        Chassis_SpinReset();
        Chassis_MechanicalUpdate(remote.channel[3] * CHASSIS_FORWARD_SCALE,
                                 remote.channel[2] * CHASSIS_LEFT_SCALE,
                                 remote.channel[0] * CHASSIS_ROTATE_SCALE);
      }
      else
      {
        Chassis_FollowReset();
        Chassis_SpinReset();
        (void)Motor3508_Stop();
      }
    }
    else
    {
      Chassis_FollowReset();
      Chassis_SpinReset();
      (void)Motor3508_Stop();
    }

    next_tick += CHASSIS_TASK_PERIOD_TICKS;
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
    /*上下板通信*/
    Communication_Service();
    taskENTER_CRITICAL();
    frame_count_snapshot = rc_task_frame_count;
    if (frame_count_snapshot > 0U)
    {
      memcpy(raw_frame, rc_task_frame, RC_FRAME_LEN);
    }
    taskEXIT_CRITICAL();

    if (frame_count_snapshot > 0U && RC_online_return())
    {
      memcpy(tx_d1, &raw_frame[0], COMMUNICATION_FRAME_SIZE);
      memcpy(tx_d2, &raw_frame[8], COMMUNICATION_FRAME_SIZE);
      memset(tx_d3, 0, sizeof(tx_d3));
      memcpy(tx_d3, &raw_frame[16], RC_FRAME_LEN - 16U);

      (void)Communication_Send(COMMUNICATION_TX_ID_D1, tx_d1);
      (void)Communication_Send(COMMUNICATION_TX_ID_D2, tx_d2);
      (void)Communication_Send(COMMUNICATION_TX_ID_D3, tx_d3);
    }

    next_tick += COMMUNICATION_TASK_PERIOD_TICKS;
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

