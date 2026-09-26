#include "motor2006.h"
#include "can.h"
#include "parameter.h"
#include <string.h>

#define MOTOR2006_CAN_FILTER_BANK        3U
#define MOTOR2006_CAN_SLAVE_START_BANK   14U
#define MOTOR2006_FILTER16(id)           ((uint32_t)(id) << 5U)
#define MOTOR2006_ENCODER_MODULUS        8192
#define MOTOR2006_ENCODER_HALF           4096

Motor2006_Feedback_t motor2006_feedback; /* 最新反馈。 */
PID_Controller_t motor2006_speed_pid;    /* 速度 PID。 */

/* CAN1 0x200 的全部电流槽必须由同一处拼接，不能分别发送并互相清零。 */
static int16_t group_current[4];
static uint32_t motor2006_last_command_ms; /* 最近一次 2006 电流命令时间。 */
static bool motor2006_command_active; /* 超时保护仅监控非零电流命令。 */

static int16_t Motor2006_LimitCurrent(int16_t current)
{
    if (current > MOTOR2006_CURRENT_LIMIT) { return MOTOR2006_CURRENT_LIMIT; }
    if (current < -MOTOR2006_CURRENT_LIMIT) { return -MOTOR2006_CURRENT_LIMIT; }
    return current;
}

static HAL_StatusTypeDef Motor2006_SendGroup(const int16_t current[4])
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[MOTOR2006_FRAME_SIZE];
    uint32_t mailbox;
    uint32_t index;

    for (index = 0U; index < 4U; index++)
    {
        uint16_t raw = (uint16_t)current[index];
        data[index * 2U] = (uint8_t)(raw >> 8U);
        data[index * 2U + 1U] = (uint8_t)raw;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) { return HAL_BUSY; }
    header.StdId = MOTOR2006_COMMAND_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = MOTOR2006_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

static HAL_StatusTypeDef Motor2006_UpdateGroup(uint32_t first_slot,
                                                 int16_t first,
                                                 int16_t second)
{
    int16_t snapshot[4];
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    group_current[first_slot] = first;
    if (first_slot == 0U) { group_current[1] = second; }
    memcpy(snapshot, group_current, sizeof(snapshot));
    __set_PRIMASK(primask);
    return Motor2006_SendGroup(snapshot);
}

HAL_StatusTypeDef Motor2006_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&motor2006_feedback, 0, sizeof(motor2006_feedback));
    memset(group_current, 0, sizeof(group_current));
    motor2006_last_command_ms = HAL_GetTick();
    motor2006_command_active = false;
    PID_Init(&motor2006_speed_pid, MOTOR2006_SPEED_KP,
             MOTOR2006_SPEED_KI, MOTOR2006_SPEED_KD,
             MOTOR2006_SPEED_INTEGRAL_LIMIT,
             MOTOR2006_SPEED_TORQUE_OUTPUT_LIMIT,
             MOTOR2006_PID_CONTROL_TIME_S);

    filter.FilterBank = MOTOR2006_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = MOTOR2006_FILTER16(MOTOR2006_FEEDBACK_CAN_ID);
    filter.FilterIdLow = filter.FilterIdHigh;
    filter.FilterMaskIdHigh = filter.FilterIdHigh;
    filter.FilterMaskIdLow = filter.FilterIdHigh;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = MOTOR2006_CAN_SLAVE_START_BANK;
    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK) { return status; }

    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Start(&hcan1);
        if (status != HAL_OK) { return status; }
    }
    else if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING)
    { return HAL_ERROR; }

    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef Motor2006_SendFrictionCurrents(int16_t left_raw,
                                                  int16_t right_raw)
{
    return Motor2006_UpdateGroup(0U, left_raw, right_raw);
}

HAL_StatusTypeDef Motor2006_SetCurrent(int16_t current_raw)
{
    int16_t limited = Motor2006_LimitCurrent(current_raw);
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    motor2006_last_command_ms = HAL_GetTick();
    motor2006_command_active = limited != 0;
    __set_PRIMASK(primask);
    return Motor2006_UpdateGroup(3U, limited, 0);
}

HAL_StatusTypeDef Motor2006_Stop(void)
{
    Motor2006_ResetSpeedPID();
    return Motor2006_SetCurrent(0);
}

HAL_StatusTypeDef Motor2006_SpeedControl(float target_rotor_rad_s)
{
    Motor2006_Feedback_t feedback;
    float rotor_rad_s;
    float torque;
    float current;

    if (!Motor2006_GetFeedback(&feedback) || !Motor2006_OnlineCheck())
    { return Motor2006_Stop(); }
    rotor_rad_s = (float)feedback.speed_rpm * 6.283185307f / 60.0f;
    torque = PID_Calc(&motor2006_speed_pid, target_rotor_rad_s, rotor_rad_s);
    current = torque / MOTOR2006_TORQUE_CONSTANT;
    if (current > MOTOR2006_CURRENT_LIMIT) { current = MOTOR2006_CURRENT_LIMIT; }
    if (current < -MOTOR2006_CURRENT_LIMIT) { current = -MOTOR2006_CURRENT_LIMIT; }
    return Motor2006_SetCurrent((int16_t)current);
}

void Motor2006_ResetSpeedPID(void)
{
    PID_Reset(&motor2006_speed_pid);
}

bool Motor2006_GetFeedback(Motor2006_Feedback_t *feedback)
{
    uint32_t primask;
    if (feedback == NULL) { return false; }
    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor2006_feedback;
    __set_PRIMASK(primask);
    return feedback->received;
}

bool Motor2006_OnlineCheck(void)
{
    Motor2006_Feedback_t feedback;
    return Motor2006_GetFeedback(&feedback) &&
           (uint32_t)(HAL_GetTick() - feedback.last_rx_ms) <
               MOTOR2006_OFFLINE_TIMEOUT_MS;
}

void Motor2006_Heartbeat(void)
{
    uint32_t now = HAL_GetTick();

    motor2006_feedback.online = Motor2006_OnlineCheck();
    if (motor2006_command_active &&
        (uint32_t)(now - motor2006_last_command_ms) >=
            MOTOR2006_COMMAND_TIMEOUT_MS)
    { (void)Motor2006_SetCurrent(0); }
}

float Motor2006_GetOutputAngleDeg(void)
{
    Motor2006_Feedback_t feedback;
    if (!Motor2006_GetFeedback(&feedback)) { return 0.0f; }
    return (float)feedback.encoder_total * 360.0f /
           (8192.0f * MOTOR2006_REDUCTION_RATIO);
}

void Motor2006_ProcessCanFrame(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               const uint8_t data[MOTOR2006_FRAME_SIZE])
{
    uint16_t encoder;
    int32_t delta;

    if (hcan == NULL || hcan->Instance != CAN1 || data == NULL ||
        std_id != MOTOR2006_FEEDBACK_CAN_ID)
    { return; }

    encoder = ((uint16_t)data[0] << 8U) | data[1];
    if (encoder >= MOTOR2006_ENCODER_MODULUS) { return; }
    if (motor2006_feedback.received)
    {
        delta = (int32_t)encoder - motor2006_feedback.last_encoder;
        if (delta > MOTOR2006_ENCODER_HALF) { delta -= MOTOR2006_ENCODER_MODULUS; }
        else if (delta < -MOTOR2006_ENCODER_HALF)
        { delta += MOTOR2006_ENCODER_MODULUS; }
        motor2006_feedback.encoder_total += delta;
    }
    motor2006_feedback.encoder = encoder;
    motor2006_feedback.last_encoder = encoder;
    motor2006_feedback.speed_rpm =
        (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    motor2006_feedback.current_raw =
        (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
    motor2006_feedback.temperature = data[6];
    motor2006_feedback.last_rx_ms = HAL_GetTick();
    motor2006_feedback.rx_count++;
    motor2006_feedback.received = true;
    motor2006_feedback.online = true;
}
