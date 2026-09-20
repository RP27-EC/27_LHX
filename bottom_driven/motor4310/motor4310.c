#include "motor4310.h"

#include "can.h"
#include <string.h>

#define MOTOR4310_CAN_FILTER_BANK        15U
#define MOTOR4310_CAN_SLAVE_START_BANK   14U
#define MOTOR4310_ANGLE_DEGREE_DIVISOR   45.111111f
#define MOTOR4310_STD_ID_TO_FILTER(id)   ((uint32_t)(id) << 5U)

Motor4310_Data_t motor4310;
PID_Controller_t motor4310_speed_pid;
PID_Controller_t motor4310_position_pid;

static HAL_StatusTypeDef Motor4310_Send(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U)
    {
        return HAL_BUSY;
    }

    header.StdId = MOTOR4310_CONTROL_CAN_ID;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = MOTOR4310_CAN_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan2, &header, (uint8_t *)data, &mailbox);
}

static HAL_StatusTypeDef Motor4310_SendSpecialCommand(uint8_t command)
{
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE];

    memset(data, 0xFF, sizeof(data));
    data[7] = command;
    return Motor4310_Send(data);
}

HAL_StatusTypeDef Motor4310_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&motor4310, 0, sizeof(motor4310));
    PID_Init(&motor4310_speed_pid,
             MOTOR4310_SPEED_KP, MOTOR4310_SPEED_KI, MOTOR4310_SPEED_KD,
             MOTOR4310_SPEED_INTEGRAL_LIMIT, MOTOR4310_SPEED_OUTPUT_LIMIT,
             MOTOR4310_CONTROL_PERIOD_S);
    PID_Init(&motor4310_position_pid,
             MOTOR4310_POSITION_KP, MOTOR4310_POSITION_KI,
             MOTOR4310_POSITION_KD, MOTOR4310_POSITION_INTEGRAL_LIMIT,
             MOTOR4310_POSITION_OUTPUT_LIMIT, MOTOR4310_CONTROL_PERIOD_S);

    /* CAN2 过滤器组 15 精确放行 Yaw 电机反馈 ID 0x012。 */
    filter.FilterBank = MOTOR4310_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh =
        MOTOR4310_STD_ID_TO_FILTER(MOTOR4310_FEEDBACK_CAN_ID);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = MOTOR4310_STD_ID_TO_FILTER(0x7FFU);
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = MOTOR4310_CAN_SLAVE_START_BANK;

    status = HAL_CAN_ConfigFilter(&hcan2, &filter);
    if (status != HAL_OK)
    {
        return status;
    }

    if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Start(&hcan2);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    else if (HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }

    status = HAL_CAN_ActivateNotification(&hcan2,
                                          CAN_IT_RX_FIFO0_MSG_PENDING);

    return status;
}

HAL_StatusTypeDef Motor4310_Enable(void)
{
    return Motor4310_SendSpecialCommand(0xFCU);
}

HAL_StatusTypeDef Motor4310_Disable(void)
{
    return Motor4310_SendSpecialCommand(0xFDU);
}

HAL_StatusTypeDef Motor4310_SetTorqueRaw(int16_t torque)
{
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE] = {0};
    uint16_t torque_code;

    if (torque > 2047)
    {
        torque = 2047;
    }
    else if (torque < -2048)
    {
        torque = -2048;
    }

    torque_code = (uint16_t)((int32_t)torque + 2048);

    /* p=0, v=0, kp=0, kd=0，仅输出转矩；字节排序与模板一致。 */
    data[0] = 0x7FU;
    data[1] = 0xFFU;
    data[2] = 0x7FU;
    data[3] = 0xF0U;
    data[6] = (uint8_t)((torque_code >> 8) & 0x0FU);
    data[7] = (uint8_t)(torque_code & 0xFFU);

    return Motor4310_Send(data);
}

HAL_StatusTypeDef Motor4310_SpeedControl(int16_t target_speed)
{
    float output;

    output = PID_Calc(&motor4310_speed_pid,
                      (float)target_speed, (float)motor4310.speed);
    return Motor4310_SetTorqueRaw((int16_t)output);
}

HAL_StatusTypeDef Motor4310_PositionControl(int32_t target_position)
{
    float target_speed;

    target_speed = PID_Calc(&motor4310_position_pid,
                            (float)target_position,
                            (float)motor4310.total_angle);
    return Motor4310_SpeedControl((int16_t)target_speed);
}

int32_t Motor4310_PositionToEcd(float rounds, float degree)
{
    return (int32_t)(rounds * MOTOR4310_ECD_PER_ROUND
                   + degree / 360.0f * MOTOR4310_ECD_PER_ROUND);
}

void Motor4310_ParseFeedback(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{
    uint16_t angle;
    uint16_t speed_code;
    uint16_t torque_code;
    int32_t delta;
    int32_t counts_per_round;

    if (data == NULL)
    {
        return;
    }

    angle = ((uint16_t)data[1] << 8) | data[2];
    speed_code = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque_code = (((uint16_t)data[4] & 0x0FU) << 8) | data[5];

    motor4310.state = data[0] >> 4;
    motor4310.angle = angle;
    motor4310.speed = (int16_t)speed_code - 2048;
    motor4310.torque = (int16_t)torque_code - 2048;
    motor4310.temperature = data[6];

    if (!motor4310.initialized)
    {
        motor4310.zero_angle = angle;
        motor4310.last_angle = angle;
        motor4310.total_angle = 0;
        motor4310.angle_degree = 0.0f;
        motor4310.initialized = true;
    }
    else
    {
        delta = (int32_t)angle - motor4310.last_angle;
        if (delta > 32767)
        {
            delta -= 65535;
        }
        else if (delta < -32767)
        {
            delta += 65535;
        }

        counts_per_round = (int32_t)(MOTOR4310_ECD_PER_ROUND + 0.5f);
        if (delta > (counts_per_round / 2))
        {
            delta -= counts_per_round;
        }
        else if (delta < -(counts_per_round / 2))
        {
            delta += counts_per_round;
        }

        motor4310.total_angle += delta;
        motor4310.last_angle = angle;
        motor4310.angle_degree =
            (float)motor4310.total_angle / MOTOR4310_ANGLE_DEGREE_DIVISOR;
    }

    motor4310.last_rx_ms = HAL_GetTick();
    motor4310.rx_count++;
}

void Motor4310_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE];

    if ((hcan == NULL) || (hcan->Instance != CAN2))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        if ((header.IDE != CAN_ID_STD) ||
            (header.RTR != CAN_RTR_DATA) ||
            (header.DLC != MOTOR4310_CAN_FRAME_SIZE) ||
            (header.StdId != MOTOR4310_FEEDBACK_CAN_ID) ||
            ((data[0] & 0x0FU) != (MOTOR4310_CONTROL_CAN_ID & 0x0FU)))
        {
            continue;
        }

        Motor4310_ParseFeedback(data);
    }
}
