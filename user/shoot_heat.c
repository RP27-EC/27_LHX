#include "shoot_heat.h"
#include <string.h>
#include <math.h>

#define HEAT_VALUE_MAX 65535.0f // 裁判热量字段的协议范围。

volatile ShootHeatState shoot_heat_state;

static bool nonnegative(float value)
{ return value >= 0.0f && value <= HEAT_VALUE_MAX; }

static bool valid_config(const ShootHeatConfig *c)
{
    return c != NULL && c->heat_per_shot > 0.0f && nonnegative(c->heat_per_shot) &&
        nonnegative(c->stop_remaining) && c->stop_remaining >= c->heat_per_shot &&
        c->low_remaining > c->stop_remaining && nonnegative(c->low_remaining) &&
        c->high_remaining > c->low_remaining && nonnegative(c->high_remaining) &&
        c->low_rate_hz > 0.0f && c->low_rate_hz <= 100.0f &&
        c->middle_rate_hz >= c->low_rate_hz && c->middle_rate_hz <= 100.0f &&
        c->high_rate_hz >= c->middle_rate_hz && c->high_rate_hz <= 100.0f &&
        nonnegative(c->offline_heat_limit) && nonnegative(c->offline_cooling_per_s) &&
        c->referee_timeout_ms > 0U && c->referee_timeout_ms < 0x80000000U &&
        c->calibration_settle_ms > 0U && c->calibration_settle_ms < 0x80000000U;
}

static void refresh_remaining(void)
{
    shoot_heat_state.remaining = fmaxf(0.0f, shoot_heat_state.heat_limit - shoot_heat_state.predicted_heat);
}

// 清空本地热量与裁判同步状态，以当前时刻建立冷却和供弹计时基准。
void ShootHeat_Init(uint32_t now_ms)
{
    memset((void *)&shoot_heat_state, 0, sizeof(shoot_heat_state));
    shoot_heat_state.last_update_ms = now_ms;
    shoot_heat_state.last_feed_ms = now_ms;
}

void ShootHeat_Update(const ShootHeatConfig *c, uint32_t now_ms,
                      bool valid, bool output_allowed, uint8_t sequence,
                      float heat, float limit, float cooling, bool feeding)
{
    float dt = (uint32_t)(now_ms - shoot_heat_state.last_update_ms) * 0.001f;
    shoot_heat_state.last_update_ms = now_ms;
    shoot_heat_state.config_valid = valid_config(c);
    if (!shoot_heat_state.config_valid)
    { shoot_heat_state.referee_valid = false; shoot_heat_state.continuous_rate_hz = 0.0f; return; }
    shoot_heat_state.predicted_heat = fmaxf(0.0f,
        shoot_heat_state.predicted_heat - shoot_heat_state.cooling_per_s * dt);
    if (feeding) { shoot_heat_state.last_feed_ms = now_ms; }
    shoot_heat_state.referee_valid = valid && nonnegative(heat) &&
        limit > 0.0f && nonnegative(limit) && nonnegative(cooling);
    shoot_heat_state.output_allowed = output_allowed;
    if (shoot_heat_state.referee_valid)
    {
        shoot_heat_state.heat_limit = limit;
        shoot_heat_state.cooling_per_s = cooling;
        if (!shoot_heat_state.sample_seen || sequence != shoot_heat_state.sample_sequence)
        {
            // 等待已在途弹丸进入裁判反馈后，才允许消除本地多计热量。
            bool settled = !feeding &&
                (uint32_t)(now_ms - shoot_heat_state.last_feed_ms) >= c->calibration_settle_ms;
            if (!shoot_heat_state.initialized || settled)
            { shoot_heat_state.predicted_heat = heat; }
            else { shoot_heat_state.predicted_heat = fmaxf(shoot_heat_state.predicted_heat, heat); }
            shoot_heat_state.referee_heat = heat;
            shoot_heat_state.sample_sequence = sequence;
            shoot_heat_state.sample_seen = true;
            shoot_heat_state.initialized = true;
            shoot_heat_state.calibration_count++;
        }
    }
    else
    {
        // 失联期间仍按最近有效冷却值维护热量；备用值只用于显式调试。
        if (!shoot_heat_state.initialized && c->allow_offline)
        {
            shoot_heat_state.heat_limit = c->offline_heat_limit;
            shoot_heat_state.cooling_per_s = c->offline_cooling_per_s;
            shoot_heat_state.initialized = true;
        }
    }
    refresh_remaining();
    shoot_heat_state.continuous_rate_hz = ShootHeat_GetRate(c);
}

bool ShootHeat_CanStart(const ShootHeatConfig *c)
{
    if (!shoot_heat_state.config_valid) { return false; }
    if (shoot_heat_state.referee_valid && !shoot_heat_state.output_allowed) { return false; }
    if (!c->enabled) { return true; }
    if (!shoot_heat_state.referee_valid && !c->allow_offline) { return false; }
    return shoot_heat_state.initialized && shoot_heat_state.remaining > c->stop_remaining;
}

void ShootHeat_RecordShot(const ShootHeatConfig *c, uint32_t now_ms)
{
    // 新供弹开始前计入，堵转重试沿用原预留。
    shoot_heat_state.predicted_heat += c->heat_per_shot;
    shoot_heat_state.last_feed_ms = now_ms;
    shoot_heat_state.predicted_shots++;
    refresh_remaining();
    shoot_heat_state.continuous_rate_hz = ShootHeat_GetRate(c);
}

float ShootHeat_GetRate(const ShootHeatConfig *c)
{
    if (!ShootHeat_CanStart(c)) { return 0.0f; }
    if (!c->enabled) { return c->high_rate_hz; }
    if (shoot_heat_state.remaining <= c->low_remaining) { return c->low_rate_hz; }
    if (shoot_heat_state.remaining < c->high_remaining) { return c->middle_rate_hz; }
    return c->high_rate_hz;
}
