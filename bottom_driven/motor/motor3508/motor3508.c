#include "motor3508.h"
#include "motor2006.h"
#include "can.h"
#include "peripheral_config.h"
#include <string.h>
#include <float.h>

#define MOTOR3508_CAN_FILTER_BANK          1U
#define MOTOR3508_CAN_SLAVE_START_BANK     14U
#define MOTOR3508_STD_ID_TO_FILTER16(id)   ((uint32_t)(id) << 5U)
#define MOTOR3508_CURRENT_RAW_MAX         16384 // C620 电流命令幅值范围。

Motor3508_Feedback_t motor3508_feedback[MOTOR3508_COUNT]; // 两个摩擦轮电机反馈。
PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT]; // 两个摩擦轮速度 PID。
volatile Motor3508BrakeState motor3508_brake_state; // 主动停轮状态。

static int16_t Motor3508_LimitCurrent(float value)
{
    int32_t limit = motor3508_config.current_limit;
    if (limit < 0) { limit = 0; }
    if (limit > MOTOR3508_CURRENT_RAW_MAX) { limit = MOTOR3508_CURRENT_RAW_MAX; }
    if (value != value) { return 0; }
    if (value > (float)limit)
    {
        return (int16_t)limit;
    }
    if (value < -(float)limit)
    {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static float Motor3508_LimitSpeed(float value)
{
    if (value > motor3508_config.max_speed_rpm)
    {
        return motor3508_config.max_speed_rpm;
    }
    if (value < -motor3508_config.max_speed_rpm)
    {
        return -motor3508_config.max_speed_rpm;
    }
    return value;
}

// 清空摩擦轮反馈，初始化两路速度 PID 并配置 CAN 反馈过滤器。
HAL_StatusTypeDef Motor3508_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    uint32_t index;

    memset(motor3508_feedback, 0, sizeof(motor3508_feedback));
    memset((void *)&motor3508_brake_state, 0, sizeof(motor3508_brake_state));
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        pid_algorithm.ops.init(&motor3508_speed_pid[index],
                 motor3508_config.speed.kp,
                 motor3508_config.speed.ki,
                 motor3508_config.speed.kd,
                 motor3508_config.speed.integral_limit,
                 motor3508_config.speed.output_limit,
                 motor3508_config.pid_control_time_s);
    }

    // 16 bit ID-list 模式精确接收两个摩擦轮 0x201/0x202。
    filter.FilterBank = MOTOR3508_CAN_FILTER_BANK;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh = MOTOR3508_STD_ID_TO_FILTER16(0x201U);
    filter.FilterIdLow = MOTOR3508_STD_ID_TO_FILTER16(0x202U);
    // 同一 bank 有四个 16 bit list 槽，后两槽重复有效 ID。
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

    // CAN1 通常已由 4310 驱动启动，仍允许本驱动独立初始化。
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
    // 0x200 群组帧第 4 槽由 M2006 使用，统一拼帧避免摩擦轮清零该槽。
    return Motor2006_SendFrictionCurrents(
        Motor3508_LimitCurrent((float)current_1),
        Motor3508_LimitCurrent((float)current_2));
}

HAL_StatusTypeDef Motor3508_SpeedControl(int16_t target_speed_rpm)
{
    float base_target;
    float target[MOTOR3508_COUNT];
    int16_t current[MOTOR3508_COUNT];
    uint32_t index;

    memset((void *)&motor3508_brake_state, 0, sizeof(motor3508_brake_state));
    base_target = Motor3508_LimitSpeed((float)target_speed_rpm);
    target[0] = base_target * motor3508_config.left_direction;
    target[1] = base_target * motor3508_config.right_direction;
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        pid_algorithm.ops.update_parameters(&motor3508_speed_pid[index],
            motor3508_config.speed.kp, motor3508_config.speed.ki,
            motor3508_config.speed.kd, motor3508_config.speed.integral_limit,
            motor3508_config.speed.output_limit,
            motor3508_config.pid_control_time_s);
        current[index] = Motor3508_LimitCurrent(
            pid_algorithm.ops.calc(&motor3508_speed_pid[index],
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
        pid_algorithm.ops.reset(&motor3508_speed_pid[index]);
    }
}

HAL_StatusTypeDef Motor3508_Stop(void)
{
    memset((void *)&motor3508_brake_state, 0, sizeof(motor3508_brake_state));
    Motor3508_ResetSpeedPID();
    return Motor3508_SendCurrent(0, 0);
}

// 零速比例闭环；停轮时清积分，制动电流始终与当前轮速相反。
HAL_StatusTypeDef Motor3508_BrakeStop(void)
{
    Motor3508BrakeConfig config = motor3508_config.brake;
    Motor3508_Feedback_t feedback;
    int16_t current[MOTOR3508_COUNT] = {0};
    uint32_t index, now = HAL_GetTick();
    Motor3508_ResetSpeedPID();
    memset((void *)&motor3508_brake_state, 0, sizeof(motor3508_brake_state));
    if (!(config.kp > 0.0f && config.kp <= FLT_MAX) ||
        config.current_limit_raw <= 0 || config.stop_speed_rpm < 0 ||
        config.stop_speed_rpm > 32767) { return Motor3508_Stop(); }
    if (config.current_limit_raw > MOTOR3508_CURRENT_RAW_MAX)
    { config.current_limit_raw = MOTOR3508_CURRENT_RAW_MAX; }
    for (index = 0U; index < MOTOR3508_COUNT; index++)
    {
        float output;
        if (!Motor3508_GetFeedback((uint8_t)(index + 1U), &feedback) ||
            (uint32_t)(now - feedback.last_rx_ms) >= motor3508_config.offline_timeout_ms)
        { continue; }
        motor3508_brake_state.feedback_valid[index] = true;
        if (feedback.speed_rpm <= config.stop_speed_rpm &&
            feedback.speed_rpm >= -config.stop_speed_rpm) { continue; }
        output = -config.kp * (float)feedback.speed_rpm;
        if (output > config.current_limit_raw) { output = (float)config.current_limit_raw; }
        if (output < -config.current_limit_raw) { output = -(float)config.current_limit_raw; }
        current[index] = Motor3508_LimitCurrent(output);
        motor3508_brake_state.active[index] = current[index] != 0;
        motor3508_brake_state.current_raw[index] = current[index];
    }
    // 共用群组拼帧，保留升降电机槽位；入队失败由下一周期继续发送。
    return Motor3508_SendCurrent(current[0], current[1]);
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
               motor3508_config.offline_timeout_ms;
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

// 绑定现有状态与函数，供外部通过模块结构体访问。
const Motor3508Module motor3508 =
{
    .config = &motor3508_config,
    .data = {
        .feedback = motor3508_feedback,
    },
    .control = {
        .speed_pid = motor3508_speed_pid,
        .brake = &motor3508_brake_state,
    },
    .init = Motor3508_Init,
    .send_current = Motor3508_SendCurrent,
    .speed_control = Motor3508_SpeedControl,
    .stop = Motor3508_Stop,
    .brake_stop = Motor3508_BrakeStop,
    .reset_speed_pid = Motor3508_ResetSpeedPID,
    .get_feedback = Motor3508_GetFeedback,
    .online_check = Motor3508_OnlineCheck,
    .all_online = Motor3508_AllOnline,
    .heartbeat = Motor3508_Heartbeat,
    .process_can_frame = Motor3508_ProcessCanFrame,
};
