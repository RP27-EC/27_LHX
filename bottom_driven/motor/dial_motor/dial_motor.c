#include "dial_motor.h"
#include "can.h"
#include "peripheral_config.h"
#include <string.h>

#define DIAL_MOTOR_CAN_FILTER_BANK        2U
#define DIAL_MOTOR_CAN_SLAVE_START_BANK   14U
#define DIAL_MOTOR_STD_ID_TO_FILTER(id)   ((uint32_t)(id) << 5U)

DialMotor_Feedback_t dial_motor_feedback; // LK4005 拨盘电机反馈快照。
volatile DialMotor_TxDiagnostics_t dial_motor_tx_diagnostics; // CAN1 发送诊断。
PID_Controller_t dial_motor_position_pid; // 拨盘累计位置外环 PID。
PID_Controller_t dial_motor_speed_pid; // 拨盘速度内环 PID。
PID_Controller_t dial_motor_continuous_speed_pid; // 连发速度环。
static uint32_t dial_motor_last_tx_ms; // 上一条控制帧入队的时间。
static bool dial_motor_tx_sent; // 上电后是否发过控制帧。

static int16_t DialMotor_LimitCurrent(int16_t current)
{
    if (current > dial_motor_config.current_limit)
    {
        return dial_motor_config.current_limit;
    }
    if (current < -dial_motor_config.current_limit)
    {
        return -dial_motor_config.current_limit;
    }
    return current;
}

static HAL_StatusTypeDef DialMotor_Send(
    const uint8_t data[DIAL_MOTOR_FRAME_SIZE])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint32_t now_ms;
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    // 保留收发间隔；未到时间留给下一控制周期重试，不阻塞发射任务。
    now_ms = HAL_GetTick();
    if ((dial_motor_feedback.received &&
         (uint32_t)(now_ms - dial_motor_feedback.last_rx_ms) <
             dial_motor_config.tx_guard_ms) ||
        (dial_motor_tx_sent &&
         (uint32_t)(now_ms - dial_motor_last_tx_ms) <
             dial_motor_config.tx_guard_ms))
    {
        dial_motor_tx_diagnostics.guard_busy_count++;
        return HAL_BUSY;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        dial_motor_tx_diagnostics.mailbox_busy_count++;
        return HAL_BUSY;
    }

    header.StdId = DIAL_MOTOR_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = DIAL_MOTOR_FRAME_SIZE;
    header.TransmitGlobalTime = DISABLE;
    status = HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
    if (status == HAL_OK)
    {
        dial_motor_last_tx_ms = HAL_GetTick();
        dial_motor_tx_sent = true;
        dial_motor_tx_diagnostics.queued_count++;
        dial_motor_tx_diagnostics.last_command = data[0];
        if (data[0] == DIAL_MOTOR_CMD_TORQUE)
        {
            dial_motor_tx_diagnostics.last_current_raw =
                (int16_t)(((uint16_t)data[5] << 8U) | data[4]);
        }
    }
    else { dial_motor_tx_diagnostics.send_error_count++; }
    return status;
}

static HAL_StatusTypeDef DialMotor_SendSimpleCommand(uint8_t command)
{
    uint8_t data[DIAL_MOTOR_FRAME_SIZE] = {0};
    data[0] = command;
    return DialMotor_Send(data);
}

// 清空拨盘反馈和发送状态，初始化位置、速度 PID 并配置 CAN 反馈过滤器。
HAL_StatusTypeDef DialMotor_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    memset(&dial_motor_feedback, 0, sizeof(dial_motor_feedback));
    memset((void *)&dial_motor_tx_diagnostics, 0,
           sizeof(dial_motor_tx_diagnostics));
    dial_motor_last_tx_ms = 0U;
    dial_motor_tx_sent = false;
    pid_algorithm.ops.init(&dial_motor_position_pid,
             dial_motor_config.position.kp,
             dial_motor_config.position.ki,
             dial_motor_config.position.kd,
             dial_motor_config.position.integral_limit,
             dial_motor_config.position.speed_limit_dps,
             dial_motor_config.pid_control_time_s);
    pid_algorithm.ops.init(&dial_motor_speed_pid,
             dial_motor_config.speed.kp,
             dial_motor_config.speed.ki,
             dial_motor_config.speed.kd,
             dial_motor_config.speed.integral_limit,
             dial_motor_config.speed.output_limit,
             dial_motor_config.pid_control_time_s);
    pid_algorithm.ops.init(&dial_motor_continuous_speed_pid,
             dial_motor_config.continuous_speed.kp,
             dial_motor_config.continuous_speed.ki,
             dial_motor_config.continuous_speed.kd,
             dial_motor_config.speed.integral_limit,
             dial_motor_config.speed.output_limit,
             dial_motor_config.pid_control_time_s);

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
    pid_algorithm.ops.reset(&dial_motor_position_pid);
    pid_algorithm.ops.reset(&dial_motor_speed_pid);
    pid_algorithm.ops.reset(&dial_motor_continuous_speed_pid);
}

HAL_StatusTypeDef DialMotor_PositionControl(int64_t target_encoder_total)
{
    return DialMotor_PositionControlLimited(target_encoder_total,
        dial_motor_config.position.speed_limit_dps);
}

HAL_StatusTypeDef DialMotor_PositionControlLimited(int64_t target_encoder_total,
                                                  float speed_limit_dps)
{
    DialMotor_Feedback_t feedback;
    float target_speed_dps;
    float current;

    if (!(speed_limit_dps > 0.0f) ||
        !DialMotor_GetFeedback(&feedback) || !DialMotor_OnlineCheck())
    {
        DialMotor_ResetControl();
        (void)DialMotor_SetTorqueCurrent(0);
        return HAL_ERROR;
    }

    if (speed_limit_dps > dial_motor_config.position.speed_limit_dps)
    { speed_limit_dps = dial_motor_config.position.speed_limit_dps; }

    pid_algorithm.ops.update_parameters(&dial_motor_position_pid,
        dial_motor_config.position.kp, dial_motor_config.position.ki,
        dial_motor_config.position.kd,
        dial_motor_config.position.integral_limit,
        speed_limit_dps,
        dial_motor_config.pid_control_time_s);
    pid_algorithm.ops.update_parameters(&dial_motor_speed_pid,
        dial_motor_config.speed.kp, dial_motor_config.speed.ki,
        dial_motor_config.speed.kd, dial_motor_config.speed.integral_limit,
        dial_motor_config.speed.output_limit,
        dial_motor_config.pid_control_time_s);
    target_speed_dps = pid_algorithm.ops.calc(&dial_motor_position_pid,
                                (float)target_encoder_total,
                                (float)feedback.encoder_total);
    current = pid_algorithm.ops.calc(&dial_motor_speed_pid,
                       target_speed_dps,
                       (float)feedback.speed_dps);
    return DialMotor_SetTorqueCurrent((int16_t)current);
}

HAL_StatusTypeDef DialMotor_SpeedControl(float target_speed_dps)
{
    DialMotor_Feedback_t feedback;
    float current;

    if (!DialMotor_GetFeedback(&feedback) || !DialMotor_OnlineCheck())
    {
        DialMotor_ResetControl();
        (void)DialMotor_SetTorqueCurrent(0);
        return HAL_ERROR;
    }

    pid_algorithm.ops.update_parameters(&dial_motor_continuous_speed_pid,
        dial_motor_config.continuous_speed.kp,
        dial_motor_config.continuous_speed.ki,
        dial_motor_config.continuous_speed.kd,
        dial_motor_config.speed.integral_limit,
        dial_motor_config.speed.output_limit,
        dial_motor_config.pid_control_time_s);
    current = pid_algorithm.ops.calc(&dial_motor_continuous_speed_pid,
                       target_speed_dps, (float)feedback.speed_dps);
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
               dial_motor_config.offline_timeout_ms;
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

    // 这类应答共用相同的温度/电流/速度/编码器布局。
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

// 绑定现有状态与函数，供外部通过模块结构体访问。
const DialMotorModule dial_motor =
{
    .config = &dial_motor_config,
    .data = {
        .feedback = &dial_motor_feedback,
        .tx = &dial_motor_tx_diagnostics,
    },
    .control = {
        .position_pid = &dial_motor_position_pid,
        .speed_pid = &dial_motor_speed_pid,
        .continuous_speed_pid = &dial_motor_continuous_speed_pid,
    },
    .init = DialMotor_Init,
    .run = DialMotor_Run,
    .stop = DialMotor_Stop,
    .close = DialMotor_Close,
    .set_torque_current = DialMotor_SetTorqueCurrent,
    .position_control = DialMotor_PositionControl,
    .position_control_limited = DialMotor_PositionControlLimited,
    .speed_control = DialMotor_SpeedControl,
    .reset_control = DialMotor_ResetControl,
    .get_feedback = DialMotor_GetFeedback,
    .online_check = DialMotor_OnlineCheck,
    .heartbeat = DialMotor_Heartbeat,
    .process_can_frame = DialMotor_ProcessCanFrame,
};
