#include "shoot_heat.h"
#include <string.h>
#include <math.h>

#define HEAT_VALUE_MAX 65535.0f // 裁判热量字段的协议范围。
#define HEAT_RESERVATION_CAPACITY 64U // 近期供弹记录容量，满后合并为保守预留。

volatile ShootHeatState shoot_heat_state;

typedef struct
{
    uint32_t time_ms;
    float heat;
} HeatReservation;

static struct
{
    HeatReservation shots[HEAT_RESERVATION_CAPACITY];
    uint32_t head;
    uint32_t count;
    float overflow_heat; // 记录满后合并的热量，等待最后一笔过期。
    uint32_t overflow_ms;
} reservations;

// 只保留反馈延迟窗口内的供弹，较早的预测误差交给裁判校准。
static void expire_reservations(const ShootHeatConfig *c, uint32_t now_ms)
{
    while (reservations.count > 0U &&
        (uint32_t)(now_ms - reservations.shots[reservations.head].time_ms) >= c->calibration_settle_ms)
    {
        shoot_heat_state.recent_reserved_heat -= reservations.shots[reservations.head].heat;
        reservations.head = (reservations.head + 1U) % HEAT_RESERVATION_CAPACITY;
        --reservations.count;
    }
    if (reservations.overflow_heat > 0.0f &&
        (uint32_t)(now_ms - reservations.overflow_ms) >= c->calibration_settle_ms)
    {
        shoot_heat_state.recent_reserved_heat -= reservations.overflow_heat;
        reservations.overflow_heat = 0.0f;
    }
    shoot_heat_state.recent_reserved_heat = reservations.count == 0U && reservations.overflow_heat == 0.0f ?
        0.0f : fmaxf(0.0f, shoot_heat_state.recent_reserved_heat);
}

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
    memset(&reservations, 0, sizeof(reservations));
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
    expire_reservations(c, now_ms);
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
            // 新样本可双向校准；近期供弹的预留覆盖裁判反馈延迟。
            bool settled = !feeding &&
                (uint32_t)(now_ms - shoot_heat_state.last_feed_ms) >= c->calibration_settle_ms;
            if (!shoot_heat_state.initialized || settled)
            { shoot_heat_state.predicted_heat = heat; }
            else
            {
                float upper_heat = heat + shoot_heat_state.recent_reserved_heat;
                shoot_heat_state.predicted_heat = fmaxf(heat,
                    fminf(shoot_heat_state.predicted_heat, upper_heat));
            }
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
    expire_reservations(c, now_ms);
    if (reservations.count < HEAT_RESERVATION_CAPACITY)
    {
        uint32_t tail = (reservations.head + reservations.count) % HEAT_RESERVATION_CAPACITY;
        reservations.shots[tail].time_ms = now_ms;
        reservations.shots[tail].heat = c->heat_per_shot;
        ++reservations.count;
    }
    else
    {
        reservations.overflow_heat += c->heat_per_shot;
        reservations.overflow_ms = now_ms;
    }
    shoot_heat_state.recent_reserved_heat += c->heat_per_shot;
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
