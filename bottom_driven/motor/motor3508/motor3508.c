#include "motor3508.h"
#include "peripheral_config.h"
#include "chassis_can.h"
#include "chassis_power.h"
#include <string.h>
#include "PID.h"
#include <float.h>

#define C620_CURRENT_RAW_MAX 16384 // C620 电流命令的协议上限。

static Motor3508_Feedback motor_feedback[MOTOR3508_COUNT]; // 四个底盘电机反馈。
static PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT]; // 四路速度环 PID。
static PID_Controller_t motor3508_position_pid[MOTOR3508_COUNT]; // 四路位置外环 PID。
static float integral_before[MOTOR3508_COUNT]; // 速度环本周期积分起点。
static float requested_current_raw[MOTOR3508_COUNT]; // PID 加前馈后的本轮电流请求。
static bool pid_pending; // 仅闭环计算后处理额外限流的积分饱和。
static uint8_t control_mode = 0U; // 0 停止，1 速度，2 位置。

// 读取单轮有效反馈，超时后停止该轮闭环计算。
static bool get_online_feedback(uint8_t motor_id, Motor3508_Feedback *feedback)
{
    return Motor3508_GetFeedback(motor_id, feedback) &&
        (uint32_t)(HAL_GetTick() - feedback->last_rx_ms) < motor3508_config.offline_timeout_ms;
}

// 清除单轮两级 PID 和积分回退记录。
static void reset_motor_pid(uint32_t index)
{
    pid_algorithm.ops.reset(&motor3508_speed_pid[index]);
    pid_algorithm.ops.reset(&motor3508_position_pid[index]);
    integral_before[index] = 0.0f;
    requested_current_raw[index] = 0.0f;
}

//模式切换清零积分
static void select_control_mode(uint8_t mode)
{
    uint32_t i;
    if (control_mode != mode)
    {
        for (i = 0U; i < MOTOR3508_COUNT; i++)
        {
            reset_motor_pid(i);
        }
        control_mode = mode;
    }
}

HAL_StatusTypeDef Motor3508_control(uint8_t mode,int16_t id1,int16_t id2,int16_t id3,int16_t id4)
{
    switch (mode) {
        case 0:
        return Motor3508_Stop();

        case 1:
        select_control_mode(mode);
        return Motor_3508_speed_control(id1,id2,id3,id4);

        case 2:
        select_control_mode(mode);
        return Motor3508_PositionControl(id1,id2,id3,id4);

        default:
        return Motor3508_Stop();
    }
}



// PID 与反馈初始化；FDCAN1 由底盘 CAN 接口启动。
HAL_StatusTypeDef Motor3508_Init(void)
{
    uint32_t i;

    memset(motor_feedback, 0, sizeof(motor_feedback));
    memset(integral_before, 0, sizeof(integral_before));
    memset(requested_current_raw, 0, sizeof(requested_current_raw));
    control_mode = 0U;
    pid_pending = false;
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        pid_algorithm.ops.init(&motor3508_position_pid[i], motor3508_config.wheel[i].position.kp,
                 motor3508_config.wheel[i].position.ki, motor3508_config.wheel[i].position.kd,
                 motor3508_config.wheel[i].position.integral_limit,
                 motor3508_config.wheel[i].position.output_limit, motor3508_config.pid_control_time_s);
    }
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        pid_algorithm.ops.init(&motor3508_speed_pid[i], motor3508_config.wheel[i].speed.kp,
                 motor3508_config.wheel[i].speed.ki, motor3508_config.wheel[i].speed.kd,
                 motor3508_config.wheel[i].speed.integral_limit,
                 motor3508_config.wheel[i].speed.output_limit, motor3508_config.pid_control_time_s);
    }
    return HAL_OK;
}

// 配置限流始终落在 C620 可发送范围内。
static int32_t current_limit_raw(void)
{
    int32_t limit = motor3508_config.current_limit;
    if (limit < 0) { limit = 0; }
    if (limit > C620_CURRENT_RAW_MAX) { limit = C620_CURRENT_RAW_MAX; }
    return limit;
}

// 浮点请求先限幅，再转电流码，避免前馈叠加后溢出。
static int16_t limit_current(float value)
{
    int32_t limit = current_limit_raw();
    if (!(value >= -FLT_MAX && value <= FLT_MAX)) { return 0; }
    if (value > (float)limit) { return (int16_t)limit; }
    if (value < -(float)limit) { return (int16_t)-limit; }
    return (int16_t)value;
}

// 固定扭矩按目标转向补偿，零速目标不主动施加前馈。
static float torque_feedforward(uint32_t index, float target_rpm)
{
    Motor3508TorqueFeedforwardConfig config = motor3508_config.wheel[index].feedforward;
    if (!(config.fixed_current_raw > 0.0f && config.fixed_current_raw <= FLT_MAX) ||
        !(config.target_deadband_rpm >= 0.0f && config.target_deadband_rpm <= FLT_MAX))
    { return 0.0f; }
    if (target_rpm > config.target_deadband_rpm) { return config.fixed_current_raw; }
    if (target_rpm < -config.target_deadband_rpm) { return -config.fixed_current_raw; }
    return 0.0f;
}

// 两种闭环共用同一前馈和电流限幅出口。
static int16_t calculate_speed_current(uint32_t index, float target_rpm, int16_t actual_rpm)
{
    integral_before[index] = motor3508_speed_pid[index].Integral;
    requested_current_raw[index] = pid_algorithm.ops.calc(&motor3508_speed_pid[index], target_rpm,
                                                         (float)actual_rpm) +
                                   torque_feedforward(index, target_rpm);
    return limit_current(requested_current_raw[index]);
}

// 四轮统一发送出口，闭环目标用于按轮分配功率。
static HAL_StatusTypeDef send_current_with_targets(int16_t id1, int16_t id2, int16_t id3, int16_t id4,
                                                    const float targets[4])
{
    int16_t values[4] = {id1, id2, id3, id4};
    uint8_t data[8];
    uint32_t i;

    for (i = 0U; i < MOTOR3508_COUNT; i++)
    { values[i] = limit_current(values[i]); }
    (void)ChassisPower_ApplyWithTargets(values, targets);
    // 直接电流命令也清除离线轮的旧闭环状态。
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        if (!chassis_power_state.wheel_feedback_valid[i]) { reset_motor_pid(i); }
    }
    if (pid_pending)
    {
        for (i = 0U; i < MOTOR3508_COUNT; i++)
        {
            PID_Controller_t *pid = &motor3508_speed_pid[i];
            float request = requested_current_raw[i];
            float limit = (float)current_limit_raw();
            if (!chassis_power_state.wheel_feedback_valid[i]) { continue; }
            bool saturated = chassis_power_state.wheel_scale[i] < 1.0f ||
                request > limit || request < -limit;
            if (saturated && (request - (float)values[i]) * pid->Ki * pid->Error > 0.0f)
            { pid->Integral = integral_before[i]; }
        }
        pid_pending = false;
    }
    // C620：标准 ID 电流控制，四个大端有符号指令。
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        uint16_t raw = (uint16_t)values[i];
        data[2U * i] = (uint8_t)(raw >> 8);
        data[2U * i + 1U] = (uint8_t)raw;
    }
    // FIFO 满、总线未启动等情况交由调用方处理，不忙等。
    return ChassisCan_Send(MOTOR3508_COMMAND_ID, data);
}

// 直接电流命令没有速度目标，使用请求功率比例分配。
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2, int16_t id3, int16_t id4)
{ return send_current_with_targets(id1, id2, id3, id4, NULL); }

//PID控速
HAL_StatusTypeDef Motor_3508_speed_control(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4){
    uint32_t i;
    const float targets[MOTOR3508_COUNT] = {speed_1, speed_2, speed_3, speed_4};
    int16_t currents[MOTOR3508_COUNT] = {0};
    Motor3508_Feedback feedback;
    select_control_mode(1U);
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        pid_algorithm.ops.update_parameters(&motor3508_speed_pid[i],
            motor3508_config.wheel[i].speed.kp, motor3508_config.wheel[i].speed.ki,
            motor3508_config.wheel[i].speed.kd, motor3508_config.wheel[i].speed.integral_limit,
            motor3508_config.wheel[i].speed.output_limit,
            motor3508_config.pid_control_time_s);
    }
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        if (!get_online_feedback((uint8_t)(i + 1U), &feedback))
        { reset_motor_pid(i); continue; }
        currents[i] = calculate_speed_current(i, targets[i], feedback.speed_rpm);
    }
    pid_pending = true;
    return send_current_with_targets(currents[0],currents[1],currents[2],currents[3], targets);
}

//4电机强制泄力
HAL_StatusTypeDef Motor3508_Stop(void)
{
    uint32_t i;
    control_mode = 0U;
    pid_pending = false;
    for (i = 0U; i < MOTOR3508_COUNT; i++) { reset_motor_pid(i); }
    return Motor3508_SendCurrent(0, 0, 0, 0);
}

//4电机控角度
HAL_StatusTypeDef Motor3508_PositionControl(float angle_1_deg,float angle_2_deg,float angle_3_deg,float angle_4_deg)
{
    const float target_angles[MOTOR3508_COUNT] = {
        angle_1_deg, angle_2_deg, angle_3_deg, angle_4_deg
    };
    Motor3508_Feedback feedback[MOTOR3508_COUNT];
    int16_t currents[MOTOR3508_COUNT] = {0, 0, 0, 0};
    uint32_t index;
    float target_speed;
    float targets[MOTOR3508_COUNT] = {0};

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        if (!(target_angles[index] >= -FLT_MAX &&
              target_angles[index] <= FLT_MAX))
        {
            (void)Motor3508_Stop();
            return HAL_ERROR;
        }
    }

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        if (!get_online_feedback((uint8_t)(index + 1U), &feedback[index]))
        { reset_motor_pid(index); continue; }
        pid_algorithm.ops.update_parameters(&motor3508_position_pid[index],
            motor3508_config.wheel[index].position.kp, motor3508_config.wheel[index].position.ki,
            motor3508_config.wheel[index].position.kd,
            motor3508_config.wheel[index].position.integral_limit,
            motor3508_config.wheel[index].position.output_limit,
            motor3508_config.pid_control_time_s);
        pid_algorithm.ops.update_parameters(&motor3508_speed_pid[index],
            motor3508_config.wheel[index].speed.kp, motor3508_config.wheel[index].speed.ki,
            motor3508_config.wheel[index].speed.kd, motor3508_config.wheel[index].speed.integral_limit,
            motor3508_config.wheel[index].speed.output_limit,
            motor3508_config.pid_control_time_s);
        target_speed = pid_algorithm.ops.calc(&motor3508_position_pid[index], target_angles[index],
                                feedback[index].position_deg);
        targets[index] = target_speed;
        currents[index] = calculate_speed_current(index, target_speed, feedback[index].speed_rpm);
    }

    pid_pending = true;
    return send_current_with_targets(currents[0], currents[1], currents[2], currents[3], targets);
}

//反馈报文获取
bool Motor3508_GetFeedback(uint8_t motor_id, Motor3508_Feedback *feedback)
{
    uint32_t saved_primask;
    if ((feedback == NULL) || (motor_id < 1U) || (motor_id > MOTOR3508_COUNT))
    {
        return false;
    }
    // 避免中断更新过程中读取到不同帧的混合字段。
    saved_primask = __get_PRIMASK();
    __disable_irq();
    *feedback = motor_feedback[motor_id - 1U];
    __set_PRIMASK(saved_primask);
    return feedback->received;
}

bool Motor3508_OnlineCheck(void){
    Motor3508_Feedback feedback;
    uint8_t motor_id;

    for (motor_id = 1U; motor_id <= MOTOR3508_COUNT; motor_id++)
    {
        if (!get_online_feedback(motor_id, &feedback))
        {
            return false;
        }
    }
    return true;
}

void Motor3508_ProcessCanFrame(uint32_t std_id, const uint8_t data[8])
{
    Motor3508_Feedback *motor;
    uint16_t encoder;
    int32_t delta;

    if (data == NULL || std_id < MOTOR3508_FEEDBACK_BASE ||
        std_id >= MOTOR3508_FEEDBACK_BASE + MOTOR3508_COUNT)
    {
        return;
    }
    encoder = ((uint16_t)data[0] << 8) | data[1];
    if (encoder > 8191U) { return; }
    motor = &motor_feedback[std_id - MOTOR3508_FEEDBACK_BASE];
    // 跨圈累计，首帧为零点；两帧间位移须小于半圈。
    if (motor->received)
    {
        delta = (int32_t)encoder - (int32_t)motor->encoder;
        if (delta > 4096) { delta -= 8192; }
        else if (delta < -4096) { delta += 8192; }
        motor->encoder_total += delta;
    }
    else
    {
        motor->encoder_total = 0;
    }
    motor->position_deg = (float)motor->encoder_total * (360.0f / 8192.0f);
    motor->encoder = encoder;
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];
    motor->last_rx_ms = HAL_GetTick();
    motor->rx_count++;
    motor->received = true;
}

// 绑定现有状态与函数，供外部通过模块结构体访问。
const Motor3508Module motor3508 =
{
    .config = &motor3508_config,
    .data = {
        .feedback = motor_feedback,
    },
    .control = {
        .position_pid = motor3508_position_pid,
        .speed_pid = motor3508_speed_pid,
        .requested_current_raw = requested_current_raw,
    },
    .execute = Motor3508_control,
    .init = Motor3508_Init,
    .send_current = Motor3508_SendCurrent,
    .speed_control = Motor_3508_speed_control,
    .position_control = Motor3508_PositionControl,
    .stop = Motor3508_Stop,
    .online_check = Motor3508_OnlineCheck,
    .get_feedback = Motor3508_GetFeedback,
    .process_can_frame = Motor3508_ProcessCanFrame,
};
