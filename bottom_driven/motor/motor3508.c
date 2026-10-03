#include "motor3508.h"
#include "peripheral_config.h"
#include "chassis_can.h"
#include "chassis_power.h"
#include <string.h>
#include "PID.h"
#include <float.h>

static Motor3508_Feedback motor_feedback[MOTOR3508_COUNT]; // 四个底盘电机反馈。
static PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT]; // 四路速度环 PID。
static PID_Controller_t motor3508_position_pid[MOTOR3508_COUNT]; // 四路位置外环 PID。
static float integral_before[MOTOR3508_COUNT]; // 速度环本周期积分起点。
static bool pid_pending; // 仅闭环计算后处理额外限流的积分饱和。
static uint8_t control_mode = 0U; // 0 停止，1 速度，2 位置。

//模式切换清零积分
static void select_control_mode(uint8_t mode)
{
    uint32_t i;
    if (control_mode != mode)
    {
        for (i = 0U; i < MOTOR3508_COUNT; i++)
        {
            PID_Reset(&motor3508_speed_pid[i]);
            PID_Reset(&motor3508_position_pid[i]);
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
    control_mode = 0U;
    pid_pending = false;
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        PID_Init(&motor3508_position_pid[i], motor3508_config.position_kp,
                 motor3508_config.position_ki, motor3508_config.position_kd,
                 motor3508_config.position_integral_limit,
                 motor3508_config.position_output_limit, motor3508_config.pid_control_time_s);
    }
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        PID_Init(&motor3508_speed_pid[i], motor3508_config.speed_kp,
                 motor3508_config.speed_ki, motor3508_config.speed_kd,
                 motor3508_config.speed_integral_limit,
                 motor3508_config.speed_output_limit, motor3508_config.pid_control_time_s);
    }
    return HAL_OK;
}

//限幅
static int16_t limit_current(int16_t value)
{
    if (value > motor3508_config.current_limit) { return motor3508_config.current_limit; }
    if (value < -motor3508_config.current_limit) { return -motor3508_config.current_limit; }
    return value;
}

//4电机力矩控制
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2,int16_t id3, int16_t id4)
{
    int16_t values[4] = {id1, id2, id3, id4};
    uint8_t data[8];
    uint32_t i;

    for (i = 0U; i < MOTOR3508_COUNT; i++)
    { values[i] = limit_current(values[i]); }
    (void)ChassisPower_Apply(values);
    if (pid_pending)
    {
        for (i = 0U; i < MOTOR3508_COUNT; i++)
        {
            PID_Controller_t *pid = &motor3508_speed_pid[i];
            bool saturated = chassis_power_state.current_scale < 1.0f ||
                pid->Output > (float)motor3508_config.current_limit ||
                pid->Output < -(float)motor3508_config.current_limit;
            if (saturated && (pid->Output - (float)values[i]) * pid->Ki * pid->Error > 0.0f)
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

//PID控速
HAL_StatusTypeDef Motor_3508_speed_control(int16_t speed_1,int16_t speed_2,int16_t speed_3,int16_t speed_4){
    uint32_t i;
    select_control_mode(1U);
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    {
        PID_UpdateParameters(&motor3508_speed_pid[i],
            motor3508_config.speed_kp, motor3508_config.speed_ki,
            motor3508_config.speed_kd, motor3508_config.speed_integral_limit,
            motor3508_config.speed_output_limit,
            motor3508_config.pid_control_time_s);
    }
    for (i = 0U; i < MOTOR3508_COUNT; i++)
    { integral_before[i] = motor3508_speed_pid[i].Integral; }
    pid_pending = true;
    int16_t id1 = PID_Calc(&motor3508_speed_pid[0],speed_1,motor_feedback[0].speed_rpm);
    int16_t id2 = PID_Calc(&motor3508_speed_pid[1],speed_2,motor_feedback[1].speed_rpm);
    int16_t id3 = PID_Calc(&motor3508_speed_pid[2],speed_3,motor_feedback[2].speed_rpm);
    int16_t id4 = PID_Calc(&motor3508_speed_pid[3],speed_4,motor_feedback[3].speed_rpm);
    return Motor3508_SendCurrent(id1,id2,id3,id4);
}

//4电机强制泄力
HAL_StatusTypeDef Motor3508_Stop(void)
{
    uint32_t i;
    control_mode = 0U;
    pid_pending = false;
    for (i = 0U; i < MOTOR3508_COUNT; i++) { PID_Reset(&motor3508_position_pid[i]); }
    for (i = 0U; i < MOTOR3508_COUNT; i++) { PID_Reset(&motor3508_speed_pid[i]); }
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
    float current;

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        if (!(target_angles[index] >= -FLT_MAX &&
              target_angles[index] <= FLT_MAX) ||
            !Motor3508_GetFeedback((uint8_t)(index + 1U), &feedback[index]))
        {
            (void)Motor3508_Stop();
            return HAL_ERROR;
        }
    }

    for (index = 0U; index < MOTOR3508_COUNT; ++index)
    {
        PID_UpdateParameters(&motor3508_position_pid[index],
            motor3508_config.position_kp, motor3508_config.position_ki,
            motor3508_config.position_kd,
            motor3508_config.position_integral_limit,
            motor3508_config.position_output_limit,
            motor3508_config.pid_control_time_s);
        PID_UpdateParameters(&motor3508_speed_pid[index],
            motor3508_config.speed_kp, motor3508_config.speed_ki,
            motor3508_config.speed_kd, motor3508_config.speed_integral_limit,
            motor3508_config.speed_output_limit,
            motor3508_config.pid_control_time_s);
        target_speed = PID_Calc(&motor3508_position_pid[index], target_angles[index],
                                feedback[index].position_deg);
        integral_before[index] = motor3508_speed_pid[index].Integral;
        current = PID_Calc(&motor3508_speed_pid[index], target_speed,
                           (float)feedback[index].speed_rpm);

        if (current > motor3508_config.current_limit) { current = motor3508_config.current_limit; }
        if (current < -motor3508_config.current_limit) { current = -motor3508_config.current_limit; }
        currents[index] = (int16_t)current;
    }

    pid_pending = true;
    return Motor3508_SendCurrent(currents[0], currents[1],currents[2], currents[3]);
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
    uint32_t now;
    uint8_t motor_id;

    now = HAL_GetTick();

    for (motor_id = 1U; motor_id <= MOTOR3508_COUNT; motor_id++)
    {
        if (!Motor3508_GetFeedback(motor_id, &feedback))
        {
            return false;
        }
        if ((uint32_t)(now - feedback.last_rx_ms) >=
            motor3508_config.offline_timeout_ms)
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
