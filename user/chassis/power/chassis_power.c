#include "chassis_power.h"
#include "application_config.h"
#include "peripheral_config.h"
#include "power_communication.h"
#include "motor3508.h"
#include "referee.h"
#include <string.h>
#include <math.h>

volatile ChassisPowerState chassis_power_state; // 模型、反馈和最终输出的观察值。
static ChassisPowerController power_controller;
static RefereeRobotStatus_t cached_robot;
static uint32_t cached_robot_ms, last_sample_count, last_sample_ms;
static bool cached_robot_valid, sample_seen;
static float last_target_w;

uint16_t ChassisPower_GetLimit(uint32_t now_ms, bool *output_allowed)
{
    RefereeRobotStatus_t robot;
    uint32_t received_ms;
    bool allowed;
    float limit;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Referee_GetRobotStatusSnapshot(&robot, &received_ms))
    { cached_robot = robot; cached_robot_ms = received_ms; cached_robot_valid = true; }
    allowed = !cached_robot_valid || cached_robot.power_management_chassis_output != 0U;
    if (cached_robot_valid && (uint32_t)(now_ms - cached_robot_ms) < REFEREE_OFFLINE_TIMEOUT_MS)
    { limit = (float)cached_robot.chassis_power_limit; }
    else
    {
        limit = chassis_power_control_config.offline_limit_w;
        if (cached_robot_valid && cached_robot.chassis_power_limit < limit)
        { limit = cached_robot.chassis_power_limit; }
    }
    __set_PRIMASK(primask);
    if (output_allowed != NULL) { *output_allowed = allowed; }
    if (!allowed || !(limit >= 0.0f && limit <= 65535.0f)) { return 0U; }
    return (uint16_t)limit;
}

// 清空功率预测和裁判缓存，以配置的初始电流比例建立反馈控制状态。
void ChassisPower_Init(void)
{
    memset((void *)&chassis_power_state, 0, sizeof(chassis_power_state));
    memset(&cached_robot, 0, sizeof(cached_robot));
    cached_robot_ms = last_sample_count = last_sample_ms = 0U;
    cached_robot_valid = sample_seen = false;
    last_target_w = -1.0f;
    chassis_power_pi_algorithm.ops.init(&power_controller, chassis_power_control_config.initial_scale);
}

float ChassisPower_Apply(int16_t currents[CHASSIS_POWER_WHEEL_COUNT])
{
    ChassisPowerControlConfig config = chassis_power_control_config;
    ChassisPowerModelConfig model = chassis_power_model_config;
    PowerCommunicationState capacitor;
    Motor3508_Feedback feedback;
    uint32_t index, now = HAL_GetTick();
    int16_t rpm[4] = {0};
    float target, scale = 1.0f, maximum = 0.0f, feedback_scale = 1.0f, model_scale = 1.0f;
    bool allowed, fresh, new_sample, motors_valid = true;
    if (currents == NULL) { return 0.0f; }
    (void)PowerCommunication_GetSnapshot(&capacitor);
    fresh = capacitor.capacitor.received &&
        (uint32_t)(now - capacitor.capacitor.last_rx_ms) < power_communication_config.offline_timeout_ms;
    target = (float)ChassisPower_GetLimit(now, &allowed) - config.reserve_w;
    if (target < 0.0f) { target = 0.0f; }
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
    {
        if (Motor3508_GetFeedback((uint8_t)(index + 1U), &feedback))
        {
            rpm[index] = feedback.speed_rpm;
            chassis_power_state.feedback_raw[index] = feedback.current_raw;
            if ((uint32_t)(now - feedback.last_rx_ms) >= motor3508_config.offline_timeout_ms)
            { motors_valid = false; }
        }
        else { motors_valid = false; chassis_power_state.feedback_raw[index] = 0; }
        if (fabsf((float)currents[index]) > maximum) { maximum = fabsf((float)currents[index]); }
        chassis_power_state.request_raw[index] = currents[index];
        chassis_power_state.speed_rpm[index] = rpm[index];
    }
    chassis_power_state.model_valid = chassis_power_model_algorithm.ops.config_valid(&model);
    chassis_power_state.control_config_valid = chassis_power_pi_algorithm.ops.config_valid(&config);
    chassis_power_state.predicted_request_w = chassis_power_state.model_valid && motors_valid ?
        chassis_power_model_algorithm.ops.predict(&model, currents, rpm, 1.0f) : 0.0f;
    new_sample = !sample_seen || last_sample_count != capacitor.capacitor.rx_count;
    if (!allowed || !chassis_power_state.control_config_valid ||
        (config.enabled && (!motors_valid || !chassis_power_state.model_valid || target == 0.0f)))
    { scale = 0.0f; model_scale = 0.0f; feedback_scale = 0.0f; }
    else if (config.enabled)
    {
        // 当前电流请求先经模型约束，再受实测反馈比例约束。
        model_scale = chassis_power_model_algorithm.ops.limit(&model, currents, rpm, target);
        if (!fresh)
        {
            feedback_scale = maximum > config.offline_current_limit ?
                config.offline_current_limit / maximum : 1.0f;
            chassis_power_pi_algorithm.ops.init(&power_controller, config.initial_scale);
            sample_seen = false;
        }
        else
        {
            if (new_sample || target != last_target_w)
            {
                float dt = new_sample ? (sample_seen ?
                    (float)(uint32_t)(capacitor.capacitor.last_rx_ms - last_sample_ms) * 0.001f : 0.0f) : 0.0f;
                float power = capacitor.capacitor.chassis_power_raw;
                if (power < 0.0f) { power = 0.0f; }
                if (maximum > 0.0f)
                { (void)chassis_power_pi_algorithm.ops.update(&power_controller, &config, power, target, dt); }
                last_sample_count = capacitor.capacitor.rx_count;
                last_sample_ms = capacitor.capacitor.last_rx_ms;
                sample_seen = true;
                last_target_w = target;
            }
            feedback_scale = power_controller.scale;
        }
        scale = fminf(model_scale, feedback_scale);
    }
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
    {
        currents[index] = (int16_t)(currents[index] * scale);
        chassis_power_state.output_raw[index] = currents[index];
    }
    chassis_power_state.predicted_output_w = chassis_power_state.model_valid && motors_valid ?
        chassis_power_model_algorithm.ops.predict(&model, currents, rpm, 1.0f) : 0.0f;
    chassis_power_state.power_w = capacitor.capacitor.chassis_power_raw;
    chassis_power_state.power_feedback_valid = fresh;
    chassis_power_state.motor_feedback_valid = motors_valid;
    chassis_power_state.referee_limit_w = cached_robot_valid ? cached_robot.chassis_power_limit : 0U;
    chassis_power_state.referee_output_allowed = allowed;
    chassis_power_state.referee_fresh = cached_robot_valid &&
        (uint32_t)(now - cached_robot_ms) < REFEREE_OFFLINE_TIMEOUT_MS;
    chassis_power_state.power_sample_count = capacitor.capacitor.rx_count;
    chassis_power_state.power_rx_ms = capacitor.capacitor.last_rx_ms;
    chassis_power_state.target_w = target;
    chassis_power_state.model_scale = model_scale;
    chassis_power_state.feedback_scale = feedback_scale;
    chassis_power_state.current_scale = scale;
    chassis_power_state.limited = maximum > 0.0f && scale < 1.0f;
    chassis_power_state.last_update_ms = now;
    return scale;
}
