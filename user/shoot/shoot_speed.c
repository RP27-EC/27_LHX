#include "shoot_speed.h"
#include <string.h>

#define SHOOT_SPEED_MAX_SAMPLE_AGE_MS 1000U // 枪管测速结果可用于调整的最长时间。
#define SHOOT_SPEED_MAX_RPM 32767 // 速度命令的正向范围。

void ShootSpeed_Init(volatile ShootSpeedState *state)
{
    if (state != NULL) { memset((void *)state, 0, sizeof(*state)); }
}

void ShootSpeed_Update(volatile ShootSpeedState *state, const ShootSpeedConfig *config,
    int32_t base_rpm, float driver_max_rpm, const ShootSpeedFeedback *feedback,
    bool ready, uint32_t now_ms)
{
    int32_t maximum, target;
    bool new_shot = false;
    if (state == NULL || config == NULL) { return; }
    maximum = config->max_speed_rpm;
    if (maximum < 0) { maximum = 0; }
    if (maximum > SHOOT_SPEED_MAX_RPM) { maximum = SHOOT_SPEED_MAX_RPM; }
    // 先检查范围再转整数，避免无效调参进入速度命令。
    if (!(driver_max_rpm > 0.0f && driver_max_rpm <= SHOOT_SPEED_MAX_RPM)) { maximum = 0; }
    else if (driver_max_rpm < maximum) { maximum = (int32_t)driver_max_rpm; }
    state->config_valid = maximum > 0 && config->step_rpm > 0 &&
        config->step_rpm <= SHOOT_SPEED_MAX_RPM &&
        config->margin_m_s >= 0.0f && config->margin_m_s < 655.0f &&
        config->deadband_m_s >= 0.0f && config->deadband_m_s < 655.0f &&
        config->fallback_limit_m_s > config->margin_m_s && config->fallback_limit_m_s < 655.0f &&
        config->feedback_timeout_ms > 0U && config->feedback_timeout_ms <= SHOOT_SPEED_MAX_SAMPLE_AGE_MS &&
        config->settle_time_ms <= SHOOT_SPEED_MAX_SAMPLE_AGE_MS;
    if (!state->initialized || base_rpm != state->manual_speed_rpm || !config->enabled) {
        state->target_speed_rpm = base_rpm;
        state->manual_speed_rpm = base_rpm;
        state->stable_since_ms = now_ms;
        state->initialized = true;
    }
    target = state->target_speed_rpm;
    if (target < 0) { target = 0; }
    if (target > maximum) { target = maximum; }
    if (target != state->target_speed_rpm) { state->stable_since_ms = now_ms; }
    state->target_speed_rpm = target;
    state->feedback_valid = feedback != NULL && feedback->valid &&
        feedback->actual_m_s > 0.0f && feedback->actual_m_s < 655.0f &&
        (uint32_t)(now_ms - feedback->received_ms) < config->feedback_timeout_ms &&
        (uint32_t)(now_ms - feedback->sample_ms) < SHOOT_SPEED_MAX_SAMPLE_AGE_MS;
    state->limit_m_s = feedback != NULL && feedback->limit_valid &&
        (uint32_t)(now_ms - feedback->received_ms) < config->feedback_timeout_ms &&
        feedback->limit_m_s > 0.0f && feedback->limit_m_s < 655.0f ?
        feedback->limit_m_s : config->fallback_limit_m_s;
    state->target_m_s = state->limit_m_s - config->margin_m_s;
    if (state->feedback_valid) { state->actual_m_s = feedback->actual_m_s; }
    // 开轮、恢复以及等待期间也消费序号，旧测速不会在放行后补调。
    if (feedback != NULL) {
        if (!state->sequence_seen || feedback->sequence != state->sequence) {
            new_shot = true;
            state->sequence = feedback->sequence;
            state->sequence_seen = true;
        }
    }
    ready = ready && config->enabled && state->config_valid;
    if (ready && !state->learning) { state->stable_since_ms = now_ms; }
    state->learning = ready;
    if (!ready || !new_shot || !state->feedback_valid || !(state->target_m_s > 0.0f)) { return; }
    // 测速必须发生在本次开轮及调速稳定以后；两板通过样本年龄对齐时间。
    if ((uint32_t)(now_ms - state->stable_since_ms) < config->settle_time_ms ||
        (uint32_t)(now_ms - feedback->sample_ms) >
        (uint32_t)(now_ms - state->stable_since_ms) - config->settle_time_ms) { return; }
    if (state->actual_m_s > state->target_m_s + config->deadband_m_s) { target -= config->step_rpm; }
    else if (state->actual_m_s < state->target_m_s - config->deadband_m_s) { target += config->step_rpm; }
    if (target < 0) { target = 0; }
    if (target > maximum) { target = maximum; }
    if (target != state->target_speed_rpm) {
        state->target_speed_rpm = target;
        state->stable_since_ms = now_ms;
        state->adjusted_count++;
    }
}
