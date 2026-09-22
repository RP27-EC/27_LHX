#include "dial_motor.h"
#include "can.h"
#include <string.h>

#define DIAL_MOTOR_CAN_FILTER_BANK        2U
#define DIAL_MOTOR_CAN_SLAVE_START_BANK   14U
#define DIAL_MOTOR_STD_ID_TO_FILTER(id)   ((uint32_t)(id) << 5U)

DialMotor_Feedback_t dial_motor_feedback;
PID_Controller_t dial_motor_position_pid;
PID_Controller_t dial_motor_speed_pid;

static int16_t DialMotor_LimitCurrent(int16_t current)
{
    if (current > DIAL_MOTOR_CURRENT_LIMIT)
    {
        return DIAL_MOTOR_CURRENT_LIMIT;
    }
    if (current < -DIAL_MOTOR_CURRENT_LIMIT)
    {
        return -DIAL_MOTOR_CURRENT_LIMIT;
    }
    return current;
}

static HAL_StatusTypeDef DialMotor_Send(
    const uint8_t data[DIAL_MOTOR_FRAME_SIZE])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if (data == NULL)
    {
        return HAL_ERROR;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return HAL_BUSY;
    }

    header.StdId = DIAL_MOTOR_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = DIAL_MOTOR_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
}

static HAL_StatusTypeDef DialMotor_SendSimpleCommand(uint8_t command)
{
    uint8_t data[DIAL_MOTOR_FRAME_SIZE] = {0};
    data[0] = command;
    return DialMotor_Send(data);
}

HAL_StatusTypeDef DialMotor_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&dial_motor_feedback, 0, sizeof(dial_motor_feedback));
    PID_Init(&dial_motor_position_pid,
             DIAL_MOTOR_POSITION_KP,
             DIAL_MOTOR_POSITION_KI,
             DIAL_MOTOR_POSITION_KD,
             DIAL_MOTOR_POSITION_INTEGRAL_LIMIT,
             DIAL_MOTOR_POSITION_SPEED_LIMIT_DPS,
             DIAL_MOTOR_PID_CONTROL_TIME_S);
    PID_Init(&dial_motor_speed_pid,
             DIAL_MOTOR_SPEED_KP,
             DIAL_MOTOR_SPEED_KI,
             DIAL_MOTOR_SPEED_KD,
             DIAL_MOTOR_SPEED_INTEGRAL_LIMIT,
             DIAL_MOTOR_SPEED_OUTPUT_LIMIT,
             DIAL_MOTOR_PID_CONTROL_TIME_S);

    filter.FilterBank = DIAL_MOTOR_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = DIAL_MOTOR_STD_ID_TO_FILTER(DIAL_MOTOR_CAN_ID);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = DIAL_MOTOR_STD_ID_TO_FILTER(0x7FFU);
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = DIAL_MOTOR_CAN_SLAVE_START_BANK;
    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

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

HAL_StatusTypeDef DialMotor_Run(void)
{
    return DialMotor_SendSimpleCommand(DIAL_MOTOR_CMD_RUN);
}

HAL_StatusTypeDef DialMotor_Stop(void)
{
    DialMotor_ResetControl();
    return DialMotor_SendSimpleCommand(DIAL_MOTOR_CMD_STOP);
}

HAL_StatusTypeDef DialMotor_Close(void)
{
    DialMotor_ResetControl();
    return DialMotor_SendSimpleCommand(DIAL_MOTOR_CMD_CLOSE);
}

HAL_StatusTypeDef DialMotor_SetTorqueCurrent(int16_t current)
{
    uint8_t data[DIAL_MOTOR_FRAME_SIZE] = {0};
    uint16_t raw;

    current = DialMotor_LimitCurrent(current);
    raw = (uint16_t)current;
    data[0] = DIAL_MOTOR_CMD_TORQUE;
    data[4] = (uint8_t)raw;
    data[5] = (uint8_t)(raw >> 8U);
    return DialMotor_Send(data);
}

void DialMotor_ResetControl(void)
{
    PID_Reset(&dial_motor_position_pid);
    PID_Reset(&dial_motor_speed_pid);
}

HAL_StatusTypeDef DialMotor_PositionControl(int64_t target_encoder_total)
{
    DialMotor_Feedback_t feedback;
    float target_speed_dps;
    float current;

    if (!DialMotor_GetFeedback(&feedback) || !DialMotor_OnlineCheck())
    {
        DialMotor_ResetControl();
        (void)DialMotor_SetTorqueCurrent(0);
        return HAL_ERROR;
    }

    target_speed_dps = PID_Calc(&dial_motor_position_pid,
                                (float)target_encoder_total,
                                (float)feedback.encoder_total);
    current = PID_Calc(&dial_motor_speed_pid,
                       target_speed_dps,
                       (float)feedback.speed_dps);
    return DialMotor_SetTorqueCurrent((int16_t)current);
}

bool DialMotor_GetFeedback(DialMotor_Feedback_t *feedback)
{
    uint32_t primask;
    if (feedback == NULL)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = dial_motor_feedback;
    __set_PRIMASK(primask);
    return feedback->received;
}

bool DialMotor_OnlineCheck(void)
{
    DialMotor_Feedback_t feedback;
    return DialMotor_GetFeedback(&feedback) &&
           (uint32_t)(HAL_GetTick() - feedback.last_rx_ms) <
               DIAL_MOTOR_OFFLINE_TIMEOUT_MS;
}

void DialMotor_Heartbeat(void)
{
    dial_motor_feedback.online = DialMotor_OnlineCheck();
}

void DialMotor_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[DIAL_MOTOR_FRAME_SIZE])
{
    int32_t delta;
    uint16_t encoder;

    if (hcan == NULL || hcan->Instance != CAN1 || data == NULL ||
        std_id != DIAL_MOTOR_CAN_ID)
    {
        return;
    }

    dial_motor_feedback.response_command = data[0];
    dial_motor_feedback.last_rx_ms = HAL_GetTick();
    dial_motor_feedback.rx_count++;
    dial_motor_feedback.received = true;
    dial_motor_feedback.online = true;

    /* 这类应答共用相同的温度/电流/速度/编码器布局。 */
    if (data[0] != DIAL_MOTOR_CMD_TORQUE && data[0] != 0xA2U &&
        (data[0] < 0xA3U || data[0] > 0xA8U))
    {
        return;
    }

    dial_motor_feedback.temperature = (int8_t)data[1];
    dial_motor_feedback.current_raw =
        (int16_t)(((uint16_t)data[3] << 8U) | data[2]);
    dial_motor_feedback.speed_dps =
        (int16_t)(((uint16_t)data[5] << 8U) | data[4]);
    encoder = ((uint16_t)data[7] << 8U) | data[6];

    if (!dial_motor_feedback.initialized)
    {
        dial_motor_feedback.encoder_total = 0;
        dial_motor_feedback.initialized = true;
    }
    else
    {
        delta = (int32_t)encoder -
                (int32_t)dial_motor_feedback.last_encoder;
        if (delta > 32767)
        {
            delta -= 65536;
        }
        else if (delta < -32768)
        {
            delta += 65536;
        }
        dial_motor_feedback.encoder_total += delta;
    }

    dial_motor_feedback.encoder = encoder;
    dial_motor_feedback.last_encoder = encoder;
    dial_motor_feedback.position_deg =
        (float)dial_motor_feedback.encoder_total *
        (360.0f / DIAL_MOTOR_ENCODER_COUNTS_PER_REV);
}
