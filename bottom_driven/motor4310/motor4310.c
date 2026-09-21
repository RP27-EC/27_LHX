#include "motor4310.h"
#include "can.h"
#include <string.h>

#define MOTOR4310_STD_ID_TO_FILTER(id) ((uint32_t)(id) << 5U)

Motor4310_Data_t motor4310_data[MOTOR4310_COUNT];
PID_Controller_t motor4310_speed_pids[MOTOR4310_COUNT];
PID_Controller_t motor4310_position_pids[MOTOR4310_COUNT];

static bool valid_id(Motor4310_Id_t id)
{
    return id == MOTOR4310_PITCH || id == MOTOR4310_YAW;
}

static CAN_HandleTypeDef *motor_can(Motor4310_Id_t id)
{
    return id == MOTOR4310_PITCH ? &hcan1 : &hcan2;
}

static uint32_t motor_tx_id(Motor4310_Id_t id)
{
    return id == MOTOR4310_PITCH ? MOTOR4310_PITCH_CONTROL_CAN_ID
                                 : MOTOR4310_CONTROL_CAN_ID;
}

static HAL_StatusTypeDef motor_send(Motor4310_Id_t id,
                                    const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{
    CAN_TxHeaderTypeDef header = {0};
    CAN_HandleTypeDef *hcan;
    uint32_t mailbox;
    if (!valid_id(id) || data == NULL) { return HAL_ERROR; }
    hcan = motor_can(id);
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) { return HAL_BUSY; }
    header.StdId = motor_tx_id(id);
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = MOTOR4310_CAN_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
}

static HAL_StatusTypeDef motor_command(Motor4310_Id_t id, uint8_t command)
{
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE];
    memset(data, 0xFF, sizeof(data));
    data[7] = command;
    return motor_send(id, data);
}

static HAL_StatusTypeDef motor_init_can(Motor4310_Id_t id)
{
    CAN_FilterTypeDef filter = {0};
    CAN_HandleTypeDef *hcan = motor_can(id);
    HAL_StatusTypeDef status;
    uint32_t rx_id = id == MOTOR4310_PITCH ?
        MOTOR4310_PITCH_FEEDBACK_CAN_ID : MOTOR4310_FEEDBACK_CAN_ID;

    /* CAN1 bank 0: Pitch; CAN2 bank 15: Yaw; bank 14: 板间通信。 */
    filter.FilterBank = id == MOTOR4310_PITCH ? 0U : 15U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = MOTOR4310_STD_ID_TO_FILTER(rx_id);
    filter.FilterMaskIdHigh = MOTOR4310_STD_ID_TO_FILTER(0x7FFU);
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    status = HAL_CAN_ConfigFilter(hcan, &filter);
    if (status != HAL_OK) { return status; }
    if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Start(hcan);
        if (status != HAL_OK) { return status; }
    }
    else if (HAL_CAN_GetState(hcan) != HAL_CAN_STATE_LISTENING)
    {
        return HAL_ERROR;
    }
    return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef Motor4310_Init(void)
{
    uint32_t i;
    memset(motor4310_data, 0, sizeof(motor4310_data));
    for (i = 0U; i < MOTOR4310_COUNT; i++)
    {
        PID_Init(&motor4310_speed_pids[i], MOTOR4310_SPEED_KP,
                 MOTOR4310_SPEED_KI, MOTOR4310_SPEED_KD,
                 MOTOR4310_SPEED_INTEGRAL_LIMIT, MOTOR4310_SPEED_OUTPUT_LIMIT,
                 MOTOR4310_CONTROL_PERIOD_S);
        PID_Init(&motor4310_position_pids[i], MOTOR4310_POSITION_KP,
                 MOTOR4310_POSITION_KI, MOTOR4310_POSITION_KD,
                 MOTOR4310_POSITION_INTEGRAL_LIMIT,
                 MOTOR4310_POSITION_OUTPUT_LIMIT, MOTOR4310_CONTROL_PERIOD_S);
    }
    if (motor_init_can(MOTOR4310_PITCH) != HAL_OK) { return HAL_ERROR; }
    return motor_init_can(MOTOR4310_YAW);
}

bool Motor4310_GetFeedback(Motor4310_Id_t id, Motor4310_Data_t *feedback)
{
    uint32_t primask;
    if (!valid_id(id) || feedback == NULL) { return false; }
    primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor4310_data[id];
    __set_PRIMASK(primask);
    return feedback->initialized;
}

bool Motor4310_OnlineCheck(Motor4310_Id_t id)
{
    Motor4310_Data_t feedback;
    return Motor4310_GetFeedback(id, &feedback) &&
           (uint32_t)(HAL_GetTick() - feedback.last_rx_ms) <
           MOTOR4310_OFFLINE_TIMEOUT_MS;
}

bool Motor4310_AllOnline(void)
{
    return Motor4310_OnlineCheck(MOTOR4310_PITCH) &&
           Motor4310_OnlineCheck(MOTOR4310_YAW);
}

void Motor4310_ResetControl(Motor4310_Id_t id)
{
    if (!valid_id(id)) { return; }
    PID_Reset(&motor4310_speed_pids[id]);
    PID_Reset(&motor4310_position_pids[id]);
}

void Motor4310_Heartbeat(void)
{
    Motor4310_Id_t id;
    static bool all_online_previous;
    bool all_online;
    for (id = MOTOR4310_PITCH; id < MOTOR4310_COUNT; id++)
    {
        bool online = Motor4310_OnlineCheck(id);
        motor4310_data[id].online = online;
    }
    all_online = motor4310_data[MOTOR4310_PITCH].online &&
                 motor4310_data[MOTOR4310_YAW].online;
    if (all_online_previous && !all_online)
    {
        /* 任意一轴断联，清掉两轴闭环状态以免恢复时突然输出。 */
        for (id = MOTOR4310_PITCH; id < MOTOR4310_COUNT; id++)
        { Motor4310_ResetControl(id); }
    }
    all_online_previous = all_online;
}

HAL_StatusTypeDef Motor4310_EnableMotor(Motor4310_Id_t id)
{
    HAL_StatusTypeDef status;
    uint32_t now;
    if (!valid_id(id)) { return HAL_ERROR; }
    now = HAL_GetTick();
    if (motor4310_data[id].enabled &&
        (Motor4310_OnlineCheck(id) ||
         (uint32_t)(now - motor4310_data[id].last_enable_ms) <
         MOTOR4310_OFFLINE_TIMEOUT_MS)) { return HAL_OK; }
    status = motor_command(id, 0xFCU);
    if (status == HAL_OK)
    {
        motor4310_data[id].enabled = true;
        motor4310_data[id].disable_command_sent = false;
        motor4310_data[id].last_enable_ms = now;
    }
    return status;
}

HAL_StatusTypeDef Motor4310_DisableMotor(Motor4310_Id_t id)
{
    HAL_StatusTypeDef status;
    uint32_t now;
    if (!valid_id(id)) { return HAL_ERROR; }
    now = HAL_GetTick();
    if (!motor4310_data[id].enabled &&
        motor4310_data[id].disable_command_sent &&
        (uint32_t)(now - motor4310_data[id].last_disable_ms) <
        MOTOR4310_DISABLE_RETRY_MS) { return HAL_OK; }
    /* CAN 入队不是电机回执：安全模式中定期重发失能命令。 */
    status = motor_command(id, 0xFDU);
    if (status == HAL_OK)
    {
        motor4310_data[id].enabled = false;
        motor4310_data[id].disable_command_sent = true;
        motor4310_data[id].last_disable_ms = now;
        Motor4310_ResetControl(id);
    }
    return status;
}

HAL_StatusTypeDef Motor4310_SetTorqueRawMotor(Motor4310_Id_t id,
                                               int16_t torque)
{
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE] = {0};
    uint16_t code;
    if (!valid_id(id) || !motor4310_data[id].enabled ||
        !Motor4310_OnlineCheck(id)) { return HAL_ERROR; }
    if (torque > 2047) { torque = 2047; }
    else if (torque < -2048) { torque = -2048; }
    code = (uint16_t)((int32_t)torque + 2048);
    data[0] = 0x7FU;
    data[1] = 0xFFU;
    data[2] = 0x7FU;
    data[3] = 0xF0U;
    data[6] = (uint8_t)((code >> 8) & 0x0FU);
    data[7] = (uint8_t)code;
    return motor_send(id, data);
}

HAL_StatusTypeDef Motor4310_SpeedControlMotor(Motor4310_Id_t id,
                                               int16_t target_speed)
{ return Motor4310_SpeedControlWithFeedforward(id, target_speed, 0); }

HAL_StatusTypeDef Motor4310_SpeedControlWithFeedforward(
    Motor4310_Id_t id, int16_t target_speed, int16_t feedforward_raw)
{
    Motor4310_Data_t feedback;
    float output;
    int32_t torque;
    if (!valid_id(id) || !Motor4310_AllOnline() ||
        !Motor4310_GetFeedback(id, &feedback)) { return HAL_ERROR; }
    output = PID_Calc(&motor4310_speed_pids[id], (float)target_speed,
                      (float)feedback.speed);
    /* 前馈由上层计算；电机驱动不关心云台机械结构。 */
    torque = (int32_t)output + feedforward_raw;
    if (torque > 2047) { torque = 2047; }
    else if (torque < -2048) { torque = -2048; }
    return Motor4310_SetTorqueRawMotor(id, (int16_t)torque);
}

HAL_StatusTypeDef Motor4310_PositionControlMotor(Motor4310_Id_t id,
                                                  int32_t target_position)
{ return Motor4310_PositionControlWithFeedforward(id, target_position, 0); }

HAL_StatusTypeDef Motor4310_PositionControlWithFeedforward(
    Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw)
{
    float speed;
    if (!valid_id(id) || !Motor4310_AllOnline()) { return HAL_ERROR; }
    speed = PID_Calc(&motor4310_position_pids[id], (float)target_position,
                     (float)motor4310_data[id].total_angle);
    return Motor4310_SpeedControlWithFeedforward(id, (int16_t)speed,
                                                  feedforward_raw);
}

HAL_StatusTypeDef Motor4310_Enable(void)
{ return Motor4310_EnableMotor(MOTOR4310_YAW); }
HAL_StatusTypeDef Motor4310_Disable(void)
{ return Motor4310_DisableMotor(MOTOR4310_YAW); }
HAL_StatusTypeDef Motor4310_SetTorqueRaw(int16_t torque)
{ return Motor4310_SetTorqueRawMotor(MOTOR4310_YAW, torque); }
HAL_StatusTypeDef Motor4310_SpeedControl(int16_t target_speed)
{ return Motor4310_SpeedControlMotor(MOTOR4310_YAW, target_speed); }
HAL_StatusTypeDef Motor4310_PositionControl(int32_t target_position)
{ return Motor4310_PositionControlMotor(MOTOR4310_YAW, target_position); }

int32_t Motor4310_PositionToEcd(float rounds, float degree)
{
    return (int32_t)(rounds * MOTOR4310_ECD_PER_ROUND +
                     degree / 360.0f * MOTOR4310_ECD_PER_ROUND);
}

void Motor4310_ParseFeedbackMotor(Motor4310_Id_t id,
    const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{
    Motor4310_Data_t *motor;
    uint16_t angle, speed_code, torque_code;
    int32_t delta, counts_per_round;
    if (!valid_id(id) || data == NULL) { return; }
    motor = &motor4310_data[id];
    angle = ((uint16_t)data[1] << 8) | data[2];
    speed_code = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque_code = (((uint16_t)data[4] & 0x0FU) << 8) | data[5];
    motor->state = data[0] >> 4;
    motor->angle = angle;
    motor->speed = (int16_t)speed_code - 2048;
    motor->torque = (int16_t)torque_code - 2048;
    motor->temperature = data[6];
    counts_per_round = (int32_t)(MOTOR4310_ECD_PER_ROUND + 0.5f);
    if (!motor->initialized)
    {
        /* 上电后以编码器数值 0 为整体零点，而不是以第一帧位置为零点。 */
        motor->zero_angle = 0U;
        motor->last_angle = angle;
        motor->total_angle = (int32_t)angle;
        /* 例如编码器 65534 应视作零点附近的 -1，而非 +65534。 */
        if (motor->total_angle > counts_per_round / 2)
        {
            motor->total_angle -= counts_per_round;
        }
        motor->initialized = true;
    }
    else
    {
        delta = (int32_t)angle - motor->last_angle;
        if (delta > counts_per_round / 2) { delta -= counts_per_round; }
        else if (delta < -(counts_per_round / 2)) { delta += counts_per_round; }
        motor->total_angle += delta;
        motor->last_angle = angle;
    }
    motor->angle_degree = (float)motor->total_angle *
                          (360.0f / MOTOR4310_ECD_PER_ROUND);
    motor->last_rx_ms = HAL_GetTick();
    motor->rx_count++;
    motor->online = true;
}

void Motor4310_ParseFeedback(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{ Motor4310_ParseFeedbackMotor(MOTOR4310_YAW, data); }

void Motor4310_ProcessCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
    const uint8_t data[MOTOR4310_CAN_FRAME_SIZE])
{
    Motor4310_Id_t id;
    if (hcan == NULL || data == NULL) { return; }
    if (hcan->Instance == CAN1 &&
        std_id == MOTOR4310_PITCH_FEEDBACK_CAN_ID) { id = MOTOR4310_PITCH; }
    else if (hcan->Instance == CAN2 &&
             std_id == MOTOR4310_FEEDBACK_CAN_ID) { id = MOTOR4310_YAW; }
    else { return; }
    if ((data[0] & 0x0FU) == (motor_tx_id(id) & 0x0FU))
    { Motor4310_ParseFeedbackMotor(id, data); }
}

void Motor4310_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[MOTOR4310_CAN_FRAME_SIZE];
    if (hcan == NULL || hcan->Instance != CAN1) { return; }
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        { break; }
        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
            header.DLC == MOTOR4310_CAN_FRAME_SIZE)
        { Motor4310_ProcessCanFrame(hcan, header.StdId, data); }
    }
}
