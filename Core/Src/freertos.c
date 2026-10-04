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
#include "application_config.h"
#include "imu.h"
#include "remote_state.h"
#include "power_communication.h"
#include "referee_uart.h"
#include "referee.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rc_task_frame[RC_FRAME_LEN]; // DBUS 帧副本。
volatile uint32_t rc_task_frame_count = 0; // 有效帧计数。
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osThreadId_t refereeTaskHandle; // 裁判解析任务。
static const osThreadAttr_t refereeTask_attributes = {
  .name = "referee",
  .stack_size = 1024,
  .priority = osPriorityNormal,
};
// Keil Watch: 0=可自旋/未选中，1=本地未布防，2=C1超时，3=上板未选中，
// 4=上板明确禁止，5=需重新布防，6=底盘锁车或电机/遥控离线，7=升降低位。
volatile uint8_t chassis_spin_block_reason = 0U;
volatile uint32_t chassis_spin_stop_count = 0U;

/* USER CODE END Variables */
/* Definitions for Control_Parsing */
osThreadId_t Control_ParsingHandle; // 遥控解析任务。
const osThreadAttr_t Control_Parsing_attributes = {
  .name = "Control_Parsing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for motor3508 */
osThreadId_t motor3508Handle; // 底盘任务。
const osThreadAttr_t motor3508_attributes = {
  .name = "motor3508",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for communication */
osThreadId_t communicationHandle; // 板间通信任务。
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void RefereeTask(void *argument);

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
  refereeTaskHandle = osThreadNew(RefereeTask, NULL, &refereeTask_attributes);
  if (refereeTaskHandle == NULL) { Error_Handler(); }
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

        // 按配置周期解析遥控并检测在线状态。
      next_tick += remote_config.task_period_ticks;

      // 超时后重建时间基准。
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
  bool lift_hold;
  bool upper_spin_selected = false;
  bool spin_allowed = false;
  bool spin_frame_valid;
  bool bottom_mode_blocked;
  bool spin_drive_enabled;
  bool spin_was_driving = false;
  bool spin_rearm_required = false;
  bool spin_session_started = false;
  bool spin_fault_timing = false;
  uint32_t spin_fault_start_ms = 0U;
  uint32_t last_wheel_speed_tx_ms = 0U;
  uint32_t last_yaw_rate_tx_ms = 0U;
  float chassis_yaw_rate_deg_s;
  bool chassis_yaw_rate_valid;
  bool wheel_feedback_valid;
  uint8_t wheel_id;
  int16_t wheel_speed_rpm[MOTOR3508_COUNT];
  Motor3508_Feedback wheel_feedback;
  (void)argument;
  next_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    // 更新底盘 IMU。
    (void)ChassisImu_Update();
    // 所有模式持续上报 D4；IMU 不可用时明确发送无效位，撤销上板前馈。
    chassis_yaw_rate_deg_s = 0.0f;
    chassis_yaw_rate_valid = ChassisImu_GetYawRate(&chassis_yaw_rate_deg_s);
    if ((uint32_t)(HAL_GetTick() - last_yaw_rate_tx_ms) >=
            chassis_config.yaw_rate_tx_period_ms &&
        Communication_SendChassisYawRateState(chassis_yaw_rate_deg_s,
                                               chassis_yaw_rate_valid) == HAL_OK)
    { last_yaw_rate_tx_ms = HAL_GetTick(); }
    RemoteState_Get(&remote);
    turn_hold = false;
    lift_hold = Communication_GetLiftLock(NULL);
    spin_frame_valid = Communication_GetSpinState(&upper_spin_selected,
                                                  &spin_allowed);
    bottom_mode_blocked = Communication_GetBottomModeBlocked();
    if (remote.mode.chassis != REMOTE_MODE_SPIN)
    {
      spin_rearm_required = false;
      spin_session_started = false;
      spin_fault_timing = false;
    }
    else if (!remote.safety.spin_enabled)
    {
      spin_rearm_required = false;
      spin_fault_timing = false;
      if (spin_frame_valid && upper_spin_selected && spin_allowed)
      { spin_session_started = true; }
    }
    else if (spin_frame_valid && upper_spin_selected && spin_allowed)
    {
      spin_session_started = true;
      spin_fault_timing = false;
    }
    else if (spin_session_started)
    {
      // 失去许可立即停转；持续失效才要求重新拨档。
      if (!spin_fault_timing)
      {
        spin_fault_timing = true;
        spin_fault_start_ms = HAL_GetTick();
      }
      else if ((uint32_t)(HAL_GetTick() - spin_fault_start_ms) >=
                    chassis_config.spin.fault_rearm_ms)
      { spin_rearm_required = true; }
    }
    if (!remote.safety.online)
    { Chassis_TurnaroundReset(remote.event.turnaround_request_count); }
    else
    {
      turn_hold = Chassis_TurnaroundUpdate(
          remote.event.turnaround_request_count,
          !(remote.mode.chassis == REMOTE_MODE_SPIN &&
            remote.safety.spin_enabled) && !lift_hold);
    }
    // 自旋由本板遥控档位和上板模式许可共同决定；Yaw 角仅用于平移坐标。
    spin_drive_enabled = remote.mode.chassis == REMOTE_MODE_SPIN &&
                         remote.safety.online && remote.safety.spin_enabled &&
                         Motor3508_OnlineCheck() && !turn_hold && !lift_hold &&
                         !bottom_mode_blocked &&
                         spin_frame_valid && upper_spin_selected &&
                         spin_allowed && !spin_rearm_required;
    if (remote.mode.chassis != REMOTE_MODE_SPIN)
    { chassis_spin_block_reason = 0U; }
    else if (!remote.safety.online || !Motor3508_OnlineCheck() ||
             turn_hold || lift_hold)
    { chassis_spin_block_reason = 6U; }
    else if (!remote.safety.spin_enabled)
    { chassis_spin_block_reason = 1U; }
    else if (bottom_mode_blocked)
    { chassis_spin_block_reason = 7U; }
    else if (!spin_frame_valid)
    { chassis_spin_block_reason = 2U; }
    else if (!upper_spin_selected)
    { chassis_spin_block_reason = 3U; }
    else if (!spin_allowed)
    { chassis_spin_block_reason = 4U; }
    else if (spin_rearm_required)
    { chassis_spin_block_reason = 5U; }
    else { chassis_spin_block_reason = 0U; }
    if (spin_was_driving && !spin_drive_enabled)
    { chassis_spin_stop_count++; }
    spin_was_driving = spin_drive_enabled;
    if (remote.safety.online && Motor3508_OnlineCheck() && !turn_hold && !lift_hold)
    {
      if (remote.mode.chassis == REMOTE_MODE_MECHANICAL ||
          (bottom_mode_blocked &&
           (remote.mode.chassis == REMOTE_MODE_FOLLOW ||
            remote.mode.chassis == REMOTE_MODE_SPIN)))
      {
        // 机械模式与升降低位强制机械模式使用同一控制分支。
        Chassis_FollowReset();
        Chassis_SpinReset();
        Chassis_MechanicalUpdate(remote.input.channel[3] * chassis_config.motion.forward_scale,
                                 remote.input.channel[2] * chassis_config.motion.left_scale,
                                 remote.input.channel[0] * chassis_config.motion.rotate_scale);
      }
      else if (remote.mode.chassis == REMOTE_MODE_SPIN)
      {
        // 小陀螺按云台朝向平移，右上档自旋。
        Chassis_FollowReset();
        Chassis_SpinUpdate(remote.input.channel[3] * chassis_config.motion.forward_scale,
                           remote.input.channel[2] * chassis_config.motion.left_scale,
                           spin_drive_enabled);
      }
      else if (remote.mode.chassis == REMOTE_MODE_FOLLOW)
      {
        Chassis_SpinReset();
        // 跟随模式由 Yaw 角驱动底盘旋转。
        Chassis_FollowUpdate(remote.input.channel[3] * chassis_config.motion.forward_scale,
                             remote.input.channel[2] * chassis_config.motion.left_scale,
                             (float)remote.input.channel[0]);
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
    wheel_feedback_valid = Motor3508_OnlineCheck();
    for (wheel_id = 1U; wheel_feedback_valid && wheel_id <= MOTOR3508_COUNT;
         wheel_id++)
    {
      if (!Motor3508_GetFeedback(wheel_id, &wheel_feedback))
      { wheel_feedback_valid = false; }
      else { wheel_speed_rpm[wheel_id - 1U] = wheel_feedback.speed_rpm; }
    }
    if (wheel_feedback_valid &&
        (uint32_t)(HAL_GetTick() - last_wheel_speed_tx_ms) >=
            chassis_config.wheel_speed_tx_period_ms &&
        Communication_SendChassisWheelSpeeds(wheel_speed_rpm) == HAL_OK)
    {
      last_wheel_speed_tx_ms = HAL_GetTick();
    }

    next_tick += chassis_config.task_period_ticks;
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
    // 转发遥控数据。
    Communication_Service();
    PowerCommunication_Service();
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

    next_tick += remote_config.period_ticks;
    if (osDelayUntil(next_tick) != osOK)
    {
      next_tick = osKernelGetTickCount();
    }
  }
  /* USER CODE END up_down_communication */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void RefereeTask(void *argument)
{
  uint32_t last_heat_tx = 0U;
  (void)argument;
  for (;;)
  {
    uint32_t now = HAL_GetTick();
    RefereeHeatSnapshot_t heat;
    RefereeUart_Process(now);
    if ((uint32_t)(now - last_heat_tx) >= remote_config.heat_tx_period_ms &&
        Referee_GetHeatSnapshot(&heat, now) &&
        Communication_SendHeatState(heat.heat, heat.limit, heat.cooling,
            heat.valid, heat.output_allowed, heat.sequence) == HAL_OK)
    { last_heat_tx = now; }
    osDelay(2U);
  }
}


/* USER CODE END Application */

