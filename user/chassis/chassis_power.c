#include "chassis_power.h"
#include "application_config.h"
#include "peripheral_config.h"
#include "power_communication.h"
#include "motor3508.h"
#include "referee.h"
#include <string.h>

volatile ChassisPowerState chassis_power_state; // Keil 展开查看，功率反馈与限流状态。

static ChassisPowerController power_controller; // 四轮共用一个功率比例闭环。
static RefereeRobotStatus_t cached_robot; // 最近的一致裁判状态。
static uint32_t cached_robot_ms, last_sample_count, last_sample_ms;
static bool cached_robot_valid, sample_seen;
static float last_target_w;

uint16_t ChassisPower_GetLimit(uint32_t now_ms, bool *output_allowed)
{
    RefereeRobotStatus_t robot;
    uint32_t received_ms, primask = __get_PRIMASK();
    bool allowed;
    float limit;
    __disable_irq();
    if (Referee_GetRobotStatusSnapshot(&robot, &received_ms))
    { cached_robot = robot; cached_robot_ms = received_ms; cached_robot_valid = true; }
    allowed = !cached_robot_valid || cached_robot.power_management_chassis_output != 0U;
    if (cached_robot_valid && (uint32_t)(now_ms - cached_robot_ms) < REFEREE_OFFLINE_TIMEOUT_MS)
    { limit = (float)cached_robot.chassis_power_limit; }
    else
    {
        limit = chassis_power_control_config.offline_limit_w;
        if (cached_robot_valid && (float)cached_robot.chassis_power_limit < limit)
        { limit = (float)cached_robot.chassis_power_limit; }
    }
    __set_PRIMASK(primask);
    if (output_allowed != NULL) { *output_allowed = allowed; }
    if (!allowed || !(limit >= 0.0f && limit <= 65535.0f)) { return 0U; }
    return (uint16_t)limit;
}

void ChassisPower_Init(void)
{
    memset((void *)&chassis_power_state, 0, sizeof(chassis_power_state));
    memset(&cached_robot, 0, sizeof(cached_robot));
    cached_robot_ms = last_sample_count = last_sample_ms = 0U;
    cached_robot_valid = sample_seen = false;
    last_target_w = -1.0f;
    ChassisPowerControl_Init(&power_controller, chassis_power_control_config.initial_scale);
}

void ChassisPower_Update(const int16_t currents[CHASSIS_POWER_WHEEL_COUNT])
{
    ChassisPowerInput input = {0};
    ChassisPowerResult result;
    ChassisPowerConfig config = chassis_power_config;
    PowerCommunicationState capacitor;
    Motor3508_Feedback feedback;
    RefereeRobotStatus_t robot;
    uint32_t i, received_ms, now_ms = HAL_GetTick();
    bool referee_fresh = false;

    if (currents == NULL) { return; }
    (void)PowerCommunication_GetSnapshot(&capacitor);
    chassis_power_state.power_w = (float)capacitor.capacitor.chassis_power_raw;
    chassis_power_state.power_feedback_valid = capacitor.capacitor.received &&
        (uint32_t)(now_ms - capacitor.capacitor.last_rx_ms) < power_communication_config.offline_timeout_ms;
    chassis_power_state.power_sample_count = capacitor.capacitor.rx_count;
    chassis_power_state.power_rx_ms = capacitor.capacitor.last_rx_ms;
    input.voltage_v = capacitor.capacitor.voltage_v;
    input.voltage_valid = chassis_power_state.power_feedback_valid &&
        capacitor.capacitor.voltage_raw >= -32000 && capacitor.capacitor.voltage_raw <= 32000 &&
        input.voltage_v >= 0.0f && input.voltage_v <= 25.0f;
    // 电压失效时不填备用电压；用有效标志区分没数据与真实零功率。
    input.feedback_valid = true;
    for (i = 0U; i < CHASSIS_POWER_WHEEL_COUNT; i++)
    {
        if (Motor3508_GetFeedback((uint8_t)(i + 1U), &feedback))
        {
            input.feedback_raw[i] = feedback.current_raw;
            if ((uint32_t)(now_ms - feedback.last_rx_ms) >= motor3508_config.offline_timeout_ms)
            { input.feedback_valid = false; }
        }
        else { input.feedback_valid = false; }
        chassis_power_state.feedback_raw[i] = input.feedback_raw[i];
        chassis_power_state.output_raw[i] = currents[i];
    }
    // 保留裁判原状态供调试，控制上限由统一快照缓存提供。
    if (Referee_GetRobotStatusSnapshot(&robot, &received_ms))
    {
        chassis_power_state.referee_limit_w = robot.chassis_power_limit;
        chassis_power_state.referee_output_allowed = robot.power_management_chassis_output != 0U;
        referee_fresh = (uint32_t)(now_ms - received_ms) < REFEREE_OFFLINE_TIMEOUT_MS;
    }
    ChassisPowerEstimate_Calculate(&config, &input, currents, &result);
    chassis_power_state.estimate = result;
    chassis_power_state.capacitor_voltage_v = input.voltage_v;
    chassis_power_state.referee_fresh = referee_fresh;
    chassis_power_state.capacitor_fresh = input.voltage_valid;
    chassis_power_state.feedback_fresh = input.feedback_valid;
    chassis_power_state.last_update_ms = now_ms;
}

float ChassisPower_Apply(int16_t currents[CHASSIS_POWER_WHEEL_COUNT])
{
    ChassisPowerControlConfig config = chassis_power_control_config;
    ChassisPowerConfig estimate_config = chassis_power_config;
    uint32_t i, now_ms = HAL_GetTick();
    float target, scale, max_current = 0.0f, dt, output_sum = 0.0f;
    bool allowed, fresh, new_sample, active;
    if (currents == NULL) { return 0.0f; }
    ChassisPower_Update(currents);
    target = (float)ChassisPower_GetLimit(now_ms, &allowed) - config.reserve_w;
    if (target < 0.0f) { target = 0.0f; }
    fresh = chassis_power_state.power_feedback_valid;
    new_sample = !sample_seen || last_sample_count != chassis_power_state.power_sample_count;
    chassis_power_state.control_config_valid = ChassisPowerControl_ConfigValid(&config);
    for (i = 0U; i < CHASSIS_POWER_WHEEL_COUNT; i++)
    {
        float magnitude = currents[i] < 0 ? -(float)currents[i] : (float)currents[i];
        if (magnitude > max_current) { max_current = magnitude; }
        chassis_power_state.request_raw[i] = currents[i];
    }
    active = max_current > 0.0f;
    if (!chassis_power_state.control_config_valid || !allowed || (config.enabled && target == 0.0f))
    { scale = 0.0f; }
    else if (!config.enabled) { scale = 1.0f; }
    else if (!fresh)
    {
        // 无反馈时不直接放开；四轮按最大指令统一缩小到备用电流。
        scale = max_current > (float)config.offline_current_limit ?
            (float)config.offline_current_limit / max_current : 1.0f;
        ChassisPowerControl_Init(&power_controller, config.initial_scale);
        sample_seen = false;
    }
    else
    {
        if (new_sample || target != last_target_w)
        {
            dt = new_sample ? (sample_seen ?
                (float)(uint32_t)(chassis_power_state.power_rx_ms - last_sample_ms) * 0.001f : 0.004f) : 0.0f;
            // 零电流时保持比例，防止停机低功率把恢复比例冲到最大。
            if (active)
            {
                float power = chassis_power_state.power_w;
                if (power < 0.0f) { power = 0.0f; } // 回馈功率不作为消耗超限。
                (void)ChassisPowerControl_Update(&power_controller, &config, power, target, dt);
            }
            last_sample_count = chassis_power_state.power_sample_count;
            last_sample_ms = chassis_power_state.power_rx_ms;
            sample_seen = true;
            last_target_w = target;
        }
        scale = power_controller.scale;
    }
    for (i = 0U; i < CHASSIS_POWER_WHEEL_COUNT; i++)
    {
        currents[i] = (int16_t)((float)currents[i] * scale);
        chassis_power_state.output_raw[i] = currents[i];
        output_sum += currents[i] < 0 ? -(float)currents[i] : (float)currents[i];
    }
    // 旧乘积诊断跟随最终电流指令，功率闭环始终只使用超电反馈。
    chassis_power_state.estimate.output_current_abs_sum_a = output_sum * estimate_config.command_a_per_raw;
    if (chassis_power_state.estimate.output_estimate_valid)
    {
        chassis_power_state.estimate.output_estimate_w = chassis_power_state.capacitor_voltage_v *
            chassis_power_state.estimate.output_current_abs_sum_a * estimate_config.estimate_gain;
    }
    chassis_power_state.target_w = target;
    chassis_power_state.current_scale = scale;
    chassis_power_state.limited = active && scale < 1.0f;
    return scale;
}
