#include "motor3508.h"
#include "can.h"
#include <string.h>

#define MOTOR3508_CAN_FILTER_BANK          1U
#define MOTOR3508_CAN_SLAVE_START_BANK     14U
#define MOTOR3508_STD_ID_TO_FILTER16(id)   ((uint32_t)(id) << 5U)

Motor3508_Feedback_t motor3508_feedback[MOTOR3508_COUNT];
PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT];

static int16_t Motor3508_LimitCurrent(float value)
{
    if (value > (float)MOTOR3508_CURRENT_LIMIT)
    {
        return (int16_t)MOTOR3508_CURRENT_LIMIT;
    }
    if (value < -(float)MOTOR3508_CURRENT_LIMIT)
    {
        return (int16_t)-MOTOR3508_CURRENT_LIMIT;
    }
    return (int16_t)value;
}

static float Motor3508_LimitSpeed(float value)
{
    if (value > MOTOR3508_MAX_SPEED_RPM)
    {
        return MOTOR3508_MAX_SPEED_RPM;
    }
    if (value < -MOTOR3508_MAX_SPEED_RPM)
    {
        return -MOTOR3508_MAX_SPEED_RPM;
    }
    return value;
}

HAL_StatusTypeDef Motor3508_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    uint32_t index;

    memset(motor3508_feedback, 0, sizeof(motor3508_feedback));
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        PID_Init(&motor3508_speed_pid[index],
                 MOTOR3508_SPEED_KP,
                 MOTOR3508_SPEED_KI,
                 MOTOR3508_SPEED_KD,
                 MOTOR3508_SPEED_INTEGRAL_LIMIT,
                 MOTOR3508_SPEED_OUTPUT_LIMIT,
                 MOTOR3508_PID_CONTROL_TIME_S);
    }

    /* 16 bit ID-list 模式精确接收两个摩擦轮 0x201/0x202。 */
    filter.FilterBank = MOTOR3508_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = MOTOR3508_STD_ID_TO_FILTER16(0x201U);
    filter.FilterIdLow = MOTOR3508_STD_ID_TO_FILTER16(0x202U);
    /* 同一 bank 有四个 16 bit list 槽，后两槽重复有效 ID。 */
    filter.FilterMaskIdHigh = MOTOR3508_STD_ID_TO_FILTER16(0x201U);
    filter.FilterMaskIdLow = MOTOR3508_STD_ID_TO_FILTER16(0x202U);
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = MOTOR3508_CAN_SLAVE_START_BANK;
    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

    /* CAN1 通常已由 4310 驱动启动，仍允许本驱动独立初始化。 */
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Start(&hcan1);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    else if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }

    return HAL_CAN_ActivateNotification(&hcan1,
                                         CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef Motor3508_SendCurrent(int16_t current_1,
                                       int16_t current_2)
{
    CAN_TxHeaderTypeDef header = {0};
    int16_t current[MOTOR3508_COUNT];
    uint8_t data[MOTOR3508_FRAME_SIZE] = {0};
    uint32_t mailbox;
    uint32_t index;

    current[0] = Motor3508_LimitCurrent((float)current_1);
    current[1] = Motor3508_LimitCurrent((float)current_2);
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        uint16_t raw = (uint16_t)current[index];
        data[index * 2U] = (uint8_t)(raw >> 8U);
        data[index * 2U + 1U] = (uint8_t)raw;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return HAL_BUSY;
    }
    header.StdId = MOTOR3508_COMMAND_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = MOTOR3508_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

HAL_StatusTypeDef Motor3508_SpeedControl(int16_t target_speed_rpm)
{
    float base_target;
    float target[MOTOR3508_COUNT];
    int16_t current[MOTOR3508_COUNT];
    uint32_t index;

    base_target = Motor3508_LimitSpeed((float)target_speed_rpm);
    target[0] = base_target * MOTOR3508_LEFT_DIRECTION;
    target[1] = base_target * MOTOR3508_RIGHT_DIRECTION;
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        current[index] = Motor3508_LimitCurrent(
            PID_Calc(&motor3508_speed_pid[index],
                     target[index],
                     (float)motor3508_feedback[index].speed_rpm));
    }
    return Motor3508_SendCurrent(current[0], current[1]);
}

void Motor3508_ResetSpeedPID(void)
{
    uint32_t index;
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        PID_Reset(&motor3508_speed_pid[index]);
    }
}

HAL_StatusTypeDef Motor3508_Stop(void)
{
    Motor3508_ResetSpeedPID();
    return Motor3508_SendCurrent(0, 0);
}

bool Motor3508_GetFeedback(uint8_t motor_id,
                           Motor3508_Feedback_t *feedback)
{
    uint32_t primask;
    if (feedback == NULL || motor_id < 1U || motor_id > MOTOR3508_COUNT)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor3508_feedback[motor_id - 1U];
    __set_PRIMASK(primask);
    return feedback->received;
}

bool Motor3508_OnlineCheck(uint8_t motor_id)
{
    Motor3508_Feedback_t feedback;
    return Motor3508_GetFeedback(motor_id, &feedback) &&
           (uint32_t)(HAL_GetTick() - feedback.last_rx_ms) <
               MOTOR3508_OFFLINE_TIMEOUT_MS;
}

bool Motor3508_AllOnline(void)
{
    uint8_t motor_id;
    for (motor_id = 1U; motor_id <= MOTOR3508_COUNT; motor_id++)
    {
        if (!Motor3508_OnlineCheck(motor_id))
        {
            return false;
        }
    }
    return true;
}

void Motor3508_Heartbeat(void)
{
    uint8_t motor_id;
    for (motor_id = 1U; motor_id <= MOTOR3508_COUNT; motor_id++)
    {
        motor3508_feedback[motor_id - 1U].online =
            Motor3508_OnlineCheck(motor_id);
    }
}

void Motor3508_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[MOTOR3508_FRAME_SIZE])
{
    Motor3508_Feedback_t *motor;
    uint32_t index;

    if (hcan == NULL || hcan->Instance != CAN1 || data == NULL ||
        std_id < MOTOR3508_FEEDBACK_BASE ||
        std_id >= MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT)
    {
        return;
    }

    index = std_id - MOTOR3508_FEEDBACK_BASE;
    motor = &motor3508_feedback[index];
    motor->encoder = ((uint16_t)data[0] << 8U) | data[1];
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    motor->current_raw = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    motor->temperature = data[6];
    motor->last_rx_ms = HAL_GetTick();
    motor->rx_count++;
    motor->received = true;
    motor->online = true;
}
