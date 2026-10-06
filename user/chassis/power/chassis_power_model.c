#include "chassis_power_model.h"
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>

#define ROTOR_RPM_TO_RAD_S 0.1047197551f // rpm 到 rad/s。
#define ALLOCATION_EPSILON_W 0.0001f // 避免分配末端的浮点残量循环。

static float clamp(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }
static bool finite_value(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }

bool ChassisPowerModel_ConfigValid(const ChassisPowerModelConfig *c)
{
    return c != NULL && c->motor.torque_nm_per_raw > 0.0f && c->motor.torque_nm_per_raw <= 0.01f &&
        c->motor.reduction_ratio >= 1.0f && c->motor.reduction_ratio <= 1000.0f &&
        c->learning.k1_min > 0.0f && c->learning.k1_max >= c->learning.k1_min && c->learning.k1_max <= 100.0f &&
        c->learning.k2_min > 0.0f && c->learning.k2_max >= c->learning.k2_min && c->learning.k2_max <= 100.0f &&
        c->initial.k1 >= c->learning.k1_min && c->initial.k1 <= c->learning.k1_max &&
        c->initial.k2 >= c->learning.k2_min && c->initial.k2 <= c->learning.k2_max &&
        c->initial.static_loss_w >= 0.0f && c->initial.static_loss_w <= 1000.0f &&
        c->learning.forgetting >= 0.9f && c->learning.forgetting <= 1.0f &&
        c->learning.initial_covariance > 0.0f && c->learning.initial_covariance <= 1000000.0f &&
        c->learning.minimum_power_w >= 0.0f && c->learning.minimum_power_w <= 65535.0f &&
        c->learning.maximum_innovation_w > 0.0f && c->learning.maximum_innovation_w <= 65535.0f &&
        c->learning.alignment_window_ms > 0U && c->learning.alignment_window_ms <= 100U &&
        c->allocation.error_blend_low_rad_s >= 0.0f &&
        c->allocation.error_blend_high_rad_s > c->allocation.error_blend_low_rad_s &&
        c->allocation.error_blend_high_rad_s <= 10000.0f;
}

// 上电或显式重置时加载模型初值，控制期间保留已学习的参数。
void ChassisPowerModel_Init(ChassisPowerModelState *state, const ChassisPowerModelConfig *config)
{
    float initial[2];
    if (state == NULL || !ChassisPowerModel_ConfigValid(config)) { return; }
    memset(state, 0, sizeof(*state));
    state->loss = config->initial;
    initial[0] = config->initial.k1; initial[1] = config->initial.k2;
    (void)rls2_algorithm.ops.init(&state->estimator, initial, config->learning.initial_covariance);
}

float ChassisPowerModel_WheelPower(const ChassisPowerModelConfig *c, const ChassisPowerLoss *loss,
                                  float current_raw, int16_t rpm)
{
    float torque = current_raw * c->motor.torque_nm_per_raw;
    float omega = rpm * ROTOR_RPM_TO_RAD_S / c->motor.reduction_ratio;
    return torque * omega + loss->k1 * fabsf(omega) + loss->k2 * torque * torque +
        loss->static_loss_w / CHASSIS_POWER_WHEEL_COUNT;
}

float ChassisPowerModel_Predict(const ChassisPowerModelConfig *c, const ChassisPowerLoss *loss,
                               const int16_t current[4], const int16_t rpm[4], const bool online[4])
{
    unsigned i;
    float sum = 0.0f;
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        float power;
        if (!online[i]) { continue; }
        power = ChassisPowerModel_WheelPower(c, loss, current[i], rpm[i]);
        if (!finite_value(power)) { return FLT_MAX; }
        if (power > 0.0f) { sum += power; }
    }
    return sum;
}

bool ChassisPowerModel_Learn(ChassisPowerModelState *state, const ChassisPowerModelConfig *c,
                            const int16_t feedback[4], const int16_t rpm[4], float measured_w)
{
    Rls2 candidate = state->estimator;
    float x[2] = {0.0f, 0.0f}, mechanical = 0.0f, y;
    unsigned i;
    state->sample_used = false;
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        float torque = feedback[i] * c->motor.torque_nm_per_raw;
        float omega = rpm[i] * ROTOR_RPM_TO_RAD_S / c->motor.reduction_ratio;
        x[0] += fabsf(omega); x[1] += torque * torque;
        mechanical += torque * omega;
    }
    state->feedback_prediction_w = mechanical + state->loss.k1 * x[0] + state->loss.k2 * x[1] +
        c->initial.static_loss_w;
    y = measured_w - mechanical - c->initial.static_loss_w;
    state->estimator.innovation = measured_w - state->feedback_prediction_w;
    // 小功率、回馈和异常残差不用于辨识，避免静止噪声拖动参数。
    if (!c->learning.enabled || !(measured_w > c->learning.minimum_power_w) ||
        !finite_value(y) || fabsf(state->estimator.innovation) > c->learning.maximum_innovation_w ||
        !rls2_algorithm.ops.update(&candidate, x, y, c->learning.forgetting))
    { state->rejected_samples++; return false; }
    candidate.theta[0] = clamp(candidate.theta[0], c->learning.k1_min, c->learning.k1_max);
    candidate.theta[1] = clamp(candidate.theta[1], c->learning.k2_min, c->learning.k2_max);
    state->estimator = candidate;
    state->loss.k1 = candidate.theta[0]; state->loss.k2 = candidate.theta[1];
    state->loss.static_loss_w = c->initial.static_loss_w;
    state->sample_used = true;
    return true;
}

void ChassisPowerModel_Allocate(ChassisPowerModelState *state, const ChassisPowerModelConfig *c,
                               int16_t current[4], const int16_t rpm[4], const bool online[4],
                               const float targets[4], float budget_w, float wheel_budget[4])
{
    float demand[4] = {0}, extra[4] = {0}, weight[4] = {0}, base[4] = {0}, error[4] = {0};
    float sum = 0.0f, base_sum = 0.0f, error_sum = 0.0f, extra_sum = 0.0f, margin = 0.0f;
    float confidence = 0.0f, available;
    unsigned i, pass;
    bool active[4] = {false};
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        float passive, a, b;
        wheel_budget[i] = 0.0f;
        if (!online[i]) { current[i] = 0; continue; }
        demand[i] = ChassisPowerModel_WheelPower(c, &state->loss, current[i], rpm[i]);
        wheel_budget[i] = fmaxf(0.0f, demand[i]);
        if (demand[i] <= 0.0f) { continue; }
        sum += demand[i];
        passive = ChassisPowerModel_WheelPower(c, &state->loss, 0.0f, rpm[i]);
        base[i] = fminf(demand[i], passive); base_sum += base[i];
        extra[i] = demand[i] - base[i]; extra_sum += extra[i];
        active[i] = extra[i] > 0.0f;
        if (active[i] && targets != NULL && finite_value(targets[i]))
        {
            error[i] = fabsf(targets[i] - rpm[i]) * ROTOR_RPM_TO_RAD_S / c->motor.reduction_ratio;
            error_sum += error[i];
        }
        a = state->loss.k2 * c->motor.torque_nm_per_raw * c->motor.torque_nm_per_raw;
        b = rpm[i] * ROTOR_RPM_TO_RAD_S / c->motor.reduction_ratio * c->motor.torque_nm_per_raw;
        margin += fabsf(b) + a * (2.0f * fabsf((float)current[i]) + 1.0f);
    }
    state->error_confidence = 0.0f;
    if (sum <= budget_w) { return; }
    available = budget_w - base_sum - margin;
    if (!(available > 0.0f) || !(extra_sum > 0.0f))
    {
        // 仅速度损耗已超限时撤销驱动电流，保留正在回馈的请求。
        for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
            if (demand[i] > 0.0f) { current[i] = 0; wheel_budget[i] = 0.0f; }
        return;
    }
    if (targets != NULL)
        confidence = clamp((error_sum - c->allocation.error_blend_low_rad_s) /
            (c->allocation.error_blend_high_rad_s - c->allocation.error_blend_low_rad_s), 0.0f, 1.0f);
    state->error_confidence = confidence;
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        wheel_budget[i] = base[i];
        if (active[i])
            weight[i] = (1.0f - confidence) * extra[i] / extra_sum +
                (error_sum > 0.0f ? confidence * error[i] / error_sum : 0.0f);
    }
    // 某轮请求已满足后，把剩余额度继续分给其他轮，最多处理全部轮数。
    for (pass = 0; pass < CHASSIS_POWER_WHEEL_COUNT; ++pass)
    {
        float sum_weight = 0.0f, spent = 0.0f;
        for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i) if (active[i]) sum_weight += weight[i];
        if (sum_weight <= 0.0f) // 零误差轮使用剩余需求比例。
        {
            for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
                if (active[i]) { weight[i] = extra[i]; sum_weight += weight[i]; }
        }
        if (sum_weight <= 0.0f || available <= ALLOCATION_EPSILON_W) { break; }
        for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
        {
            float granted;
            if (!active[i]) { continue; }
            granted = fminf(extra[i], available * weight[i] / sum_weight);
            wheel_budget[i] += granted; extra[i] -= granted; spent += granted;
            if (extra[i] <= ALLOCATION_EPSILON_W) { active[i] = false; }
        }
        available -= spent;
    }
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        float a, b, passive, discriminant, result;
        if (!online[i] || demand[i] <= wheel_budget[i] || demand[i] <= 0.0f) { continue; }
        a = state->loss.k2 * c->motor.torque_nm_per_raw * c->motor.torque_nm_per_raw;
        b = rpm[i] * ROTOR_RPM_TO_RAD_S / c->motor.reduction_ratio * c->motor.torque_nm_per_raw;
        passive = ChassisPowerModel_WheelPower(c, &state->loss, 0.0f, rpm[i]);
        discriminant = b * b + 4.0f * a * (wheel_budget[i] - passive);
        if (!(discriminant >= 0.0f)) { current[i] = 0; continue; }
        result = (-b + (current[i] >= 0 ? 1.0f : -1.0f) * sqrtf(discriminant)) / (2.0f * a);
        if (!finite_value(result) || result * current[i] < 0.0f) { result = 0.0f; }
        result = clamp(result, -fabsf((float)current[i]), fabsf((float)current[i]));
        current[i] = (int16_t)result;
    }
}

const ChassisPowerModelAlgorithm chassis_power_model_algorithm = {
    .ops = {.config_valid = ChassisPowerModel_ConfigValid, .init = ChassisPowerModel_Init,
            .predict = ChassisPowerModel_Predict, .learn = ChassisPowerModel_Learn,
            .allocate = ChassisPowerModel_Allocate}
};
