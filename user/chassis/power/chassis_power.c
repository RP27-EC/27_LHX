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
ChassisPowerModelState chassis_power_model_state; // 在线损耗参数及辨识历史。
static ChassisPowerModelConfig model_config_snapshot;
static bool model_config_seen;

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

// 清除裁判缓存和辨识历史，预算修正从完整目标功率开始。
void ChassisPower_Init(void)
{
    ChassisPowerModelConfig config = chassis_power_model_config;
    memset((void *)&chassis_power_state, 0, sizeof(chassis_power_state));
    memset(&cached_robot, 0, sizeof(cached_robot));
    memset(&chassis_power_model_state, 0, sizeof(chassis_power_model_state));
    cached_robot_ms = last_sample_count = last_sample_ms = 0U;
    cached_robot_valid = sample_seen = model_config_seen = false;
    last_target_w = -1.0f;
    if (chassis_power_model_algorithm.ops.config_valid(&config))
    {
        chassis_power_model_algorithm.ops.init(&chassis_power_model_state, &config);
        model_config_snapshot = config; model_config_seen = true;
    }
    chassis_power_pi_algorithm.ops.init(&power_controller, 1.0f);
}

// 使用双向无符号时间差，允许 HAL 毫秒计数回绕。
static bool samples_aligned(uint32_t motor_ms, uint32_t power_ms, uint32_t window_ms)
{
    return (uint32_t)(motor_ms - power_ms) <= window_ms ||
        (uint32_t)(power_ms - motor_ms) <= window_ms;
}

float ChassisPower_Apply(int16_t currents[CHASSIS_POWER_WHEEL_COUNT])
{ return ChassisPower_ApplyWithTargets(currents, NULL); }

float ChassisPower_ApplyWithTargets(int16_t currents[CHASSIS_POWER_WHEEL_COUNT], const float targets[4])
{
    ChassisPowerControlConfig config = chassis_power_control_config;
    ChassisPowerModelConfig model = chassis_power_model_config;
    PowerCommunicationState capacitor;
    Motor3508_Feedback feedback;
    uint32_t index, now;
    int16_t rpm[4] = {0}, actual[4] = {0}, request[4];
    bool online[4] = {false}, aligned = true, allowed, fresh, new_sample, motors_valid = true;
    float wheel_budget[4] = {0}, target, budget, minimum_scale = 1.0f, model_scale = 1.0f;
    float feedback_scale = 1.0f;
    bool model_valid, control_valid;
    if (currents == NULL) { return 0.0f; }
    (void)PowerCommunication_GetSnapshot(&capacitor);
    now = HAL_GetTick();
    fresh = capacitor.capacitor.received &&
        (uint32_t)(now - capacitor.capacitor.last_rx_ms) < power_communication_config.offline_timeout_ms;
    new_sample = fresh && (!sample_seen || last_sample_count != capacitor.capacitor.rx_count);
    target = (float)ChassisPower_GetLimit(now, &allowed) - config.reserve_w;
    if (!(target > 0.0f)) { target = 0.0f; }
    model_valid = chassis_power_model_algorithm.ops.config_valid(&model);
    control_valid = chassis_power_pi_algorithm.ops.config_valid(&config);
    if (model_valid && (!model_config_seen ||
        model.motor.torque_nm_per_raw != model_config_snapshot.motor.torque_nm_per_raw ||
        model.motor.reduction_ratio != model_config_snapshot.motor.reduction_ratio ||
        model.initial.k1 != model_config_snapshot.initial.k1 ||
        model.initial.k2 != model_config_snapshot.initial.k2 ||
        model.initial.static_loss_w != model_config_snapshot.initial.static_loss_w ||
        model.learning.initial_covariance != model_config_snapshot.learning.initial_covariance))
    {
        chassis_power_model_algorithm.ops.init(&chassis_power_model_state, &model);
        model_config_seen = true;
    }
    if (model_valid)
    {
        // 在线调整边界保留学习历史，参数立即落到新的有效范围内。
        chassis_power_model_state.loss.k1 = fminf(model.learning.k1_max,
            fmaxf(model.learning.k1_min, chassis_power_model_state.loss.k1));
        chassis_power_model_state.loss.k2 = fminf(model.learning.k2_max,
            fmaxf(model.learning.k2_min, chassis_power_model_state.loss.k2));
        chassis_power_model_state.estimator.theta[0] = chassis_power_model_state.loss.k1;
        chassis_power_model_state.estimator.theta[1] = chassis_power_model_state.loss.k2;
        model_config_snapshot = model;
    }
    chassis_power_model_state.sample_used = false;
    chassis_power_model_state.error_confidence = 0.0f;
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
    {
        chassis_power_state.request_raw[index] = request[index] = currents[index];
        online[index] = Motor3508_GetFeedback((uint8_t)(index + 1U), &feedback) &&
            (uint32_t)(HAL_GetTick() - feedback.last_rx_ms) < motor3508_config.offline_timeout_ms;
        if (online[index])
        {
            rpm[index] = feedback.speed_rpm; actual[index] = feedback.current_raw;
            aligned = aligned && samples_aligned(feedback.last_rx_ms, capacitor.capacitor.last_rx_ms,
                                                model.learning.alignment_window_ms);
        }
        else
        {
            motors_valid = aligned = false;
            currents[index] = 0; // 离线轮单独停机，其旧反馈不参与分配。
        }
        chassis_power_state.wheel_feedback_valid[index] = online[index];
        chassis_power_state.speed_rpm[index] = rpm[index];
        chassis_power_state.feedback_raw[index] = actual[index];
        chassis_power_state.target_speed_rpm[index] = targets != NULL ? targets[index] : 0.0f;
    }
    // 每个超电样本最多学习一次，使用实际反馈电流而非待发送电流。
    if (new_sample && model_valid && control_valid && config.enabled)
    {
        if (aligned)
            (void)chassis_power_model_algorithm.ops.learn(&chassis_power_model_state, &model, actual, rpm,
                                                         capacitor.capacitor.chassis_power_raw);
        else { chassis_power_model_state.rejected_samples++; }
    }
    chassis_power_state.predicted_request_w = model_valid ?
        chassis_power_model_algorithm.ops.predict(&model, &chassis_power_model_state.loss, currents, rpm, online) : 0.0f;
    if (!fresh || !config.enabled)
    {
        chassis_power_pi_algorithm.ops.init(&power_controller, 1.0f);
    }
    else if (config.enabled && control_valid && (new_sample || target != last_target_w))
    {
        float dt = new_sample && sample_seen ?
            (float)(uint32_t)(capacitor.capacitor.last_rx_ms - last_sample_ms) * 0.001f : 0.0f;
        float power = fmaxf(0.0f, capacitor.capacitor.chassis_power_raw);
        (void)chassis_power_pi_algorithm.ops.update(&power_controller, &config, power, target, dt);
    }
    feedback_scale = fresh && config.enabled ? power_controller.scale : 1.0f;
    budget = target * feedback_scale;
    if (!allowed || !control_valid || (config.enabled && (!model_valid || target == 0.0f)))
    {
        memset(currents, 0, sizeof(int16_t) * CHASSIS_POWER_WHEEL_COUNT);
        budget = 0.0f; model_scale = 0.0f;
        // 禁用期间清除预算积分，恢复许可后重新使用完整模型预算。
        chassis_power_pi_algorithm.ops.init(&power_controller, 1.0f);
    }
    else if (config.enabled)
    {
        // 实测反馈修正总预算，分配器根据各轮误差分别求允许电流。
        chassis_power_model_algorithm.ops.allocate(&chassis_power_model_state, &model, currents, rpm,
                                                   online, targets, budget, wheel_budget);
        for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
        {
            float ratio = request[index] != 0 ? fabsf((float)currents[index] / request[index]) : 1.0f;
            if (online[index] && ratio < model_scale) { model_scale = ratio; }
            if (!fresh)
            {
                if (currents[index] > config.offline_current_limit) { currents[index] = (int16_t)config.offline_current_limit; }
                if (currents[index] < -config.offline_current_limit) { currents[index] = (int16_t)-config.offline_current_limit; }
            }
        }
    }
    // 收紧单轮电流可能减弱制动，最后确认正功率仍在可行额度内。
    if (config.enabled && model_valid && !fresh)
    {
        float predicted = chassis_power_model_algorithm.ops.predict(&model, &chassis_power_model_state.loss, currents, rpm, online);
        if (predicted > budget)
        {
            // 失联降级采用逐轮零电流，避免陈旧功率支撑制动功率抵扣。
            for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
                if (online[index] && ChassisPowerModel_WheelPower(&model, &chassis_power_model_state.loss,
                                                                currents[index], rpm[index]) > 0.0f)
                { currents[index] = 0; }
        }
    }
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
    {
        float ratio = request[index] != 0 ? fabsf((float)currents[index] / request[index]) : 1.0f;
        chassis_power_state.wheel_scale[index] = ratio;
        chassis_power_state.wheel_budget_w[index] = wheel_budget[index];
        chassis_power_state.output_raw[index] = currents[index];
        if (online[index] && ratio < minimum_scale) { minimum_scale = ratio; }
    }
    if (fresh)
    {
        if (new_sample) { last_sample_ms = capacitor.capacitor.last_rx_ms; }
        last_sample_count = capacitor.capacitor.rx_count; sample_seen = true;
    }
    else { sample_seen = false; }
    last_target_w = target;
    chassis_power_state.predicted_output_w = model_valid ?
        chassis_power_model_algorithm.ops.predict(&model, &chassis_power_model_state.loss, currents, rpm, online) : 0.0f;
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
    chassis_power_state.allocation_budget_w = budget;
    chassis_power_state.model_scale = model_scale;
    chassis_power_state.feedback_scale = feedback_scale;
    chassis_power_state.current_scale = minimum_scale;
    chassis_power_state.speed_targets_valid = targets != NULL;
    chassis_power_state.learning_sample_aligned = new_sample && aligned;
    chassis_power_state.model_valid = model_valid;
    chassis_power_state.control_config_valid = control_valid;
    chassis_power_state.limited = false;
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
        if (request[index] != currents[index]) { chassis_power_state.limited = true; }
    chassis_power_state.last_update_ms = now;
    return minimum_scale;
}
