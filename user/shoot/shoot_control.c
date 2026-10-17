#include "shoot_control.h"
#include "dial_motor.h"
#include "motor3508.h"
#include "application_config.h"
#include "peripheral_config.h"
#include "stm32f4xx_hal.h"
#include "communication.h"
#include <math.h>

#define SHOOT_SINGLE_APPEND_PER_CYCLE 16U // 限制单周期追加目标和计热的工作量。

// 集中保存拨盘、堵转、停机及键鼠单发状态，便于调试观察。
volatile ShootControlState_t shoot_control_state = {
    .dial.state = SHOOT_DIAL_IDLE,
};

static bool idle_hold_enabled = true; // 遥控和拨盘在线时允许待机保持。
static bool continuous_tracking;
static int64_t continuous_origin; // 连发开始的累计位置。
static uint32_t continuous_reserved; // 已计热的供弹圈数，退让不撤销。
static void Shoot_DialIdleHold(void);

static void Shoot_HeatUpdate(void)
{
    Communication_HeatSnapshot_t heat = {0};
    ShootHeatConfig config = shoot_heat_config;
    uint32_t now = HAL_GetTick();
    bool valid = Communication_GetHeatSnapshot(&heat) && heat.valid &&
        (uint32_t)(now - heat.last_rx_ms) < config.referee_timeout_ms;
    bool feeding = shoot_control_state.dial.state != SHOOT_DIAL_IDLE;
    ShootHeat_Update(&config, now, valid, heat.output_allowed, heat.sequence,
                     heat.heat, heat.limit, heat.cooling, feeding);
}

static bool Shoot_HeatCanStart(void)
{
    ShootHeatConfig config = shoot_heat_config;
    return ShootHeat_CanStart(&config);
}

static void Shoot_HeatReserve(void)
{
    ShootHeatConfig config = shoot_heat_config;
    ShootHeat_RecordShot(&config, HAL_GetTick());
}

static int32_t Shoot_AbsInt32(int32_t value)
{
    return value < 0 ? -value : value;
}

// 关闭清本次重试，累计次数保留供调试查看。
static void Shoot_FricReset(void)
{
    shoot_control_state.friction.state = SHOOT_FRIC_NORMAL;
    shoot_control_state.friction.enabled = false;
    shoot_control_state.friction.attempts = 0;
    shoot_control_state.friction.block_timing[0] = false;
    shoot_control_state.friction.block_timing[1] = false;
}

static bool Shoot_FricConfigValid(void)
{
    return shoot_config.friction.boost_duration_ms > 0 &&
        shoot_config.friction.boost_duration_ms <= 1000U &&
        shoot_config.friction.boost_current_raw > 0 && shoot_config.friction.boost_current_raw <= 16384 &&
        shoot_config.friction.block_current_raw > 0 && shoot_config.friction.block_speed_rpm > 0 &&
        shoot_config.friction.block_confirm_ms > 0 && shoot_config.friction.recovery_max_attempts > 0 &&
        shoot_config.friction.target_speed_rpm > shoot_config.friction.block_speed_rpm &&
        (motor3508_config.left_direction == 1 || motor3508_config.left_direction == -1) &&
        (motor3508_config.right_direction == 1 || motor3508_config.right_direction == -1);
}

static bool Shoot_FricPrepare(bool enabled)
{
    uint32_t now = HAL_GetTick(), i;
    Motor3508_Feedback_t feedback;
    if (!enabled) { Shoot_FricReset(); return false; }
    if (!shoot_control_state.friction.enabled) {
        Shoot_FricReset();
        shoot_control_state.friction.enabled = true;
        shoot_control_state.friction.state_start_ms = now;
    }
    if (!Motor3508_AllOnline()) { shoot_control_state.friction.state = SHOOT_FRIC_FAULT; }
    if (shoot_control_state.friction.state == SHOOT_FRIC_FAULT) { return false; }
    if (shoot_control_state.friction.state == SHOOT_FRIC_BOOST) {
        if (!Shoot_FricConfigValid()) {
            shoot_control_state.friction.state = SHOOT_FRIC_FAULT;
            return false;
        }
        if ((uint32_t)(now - shoot_control_state.friction.state_start_ms) <
            shoot_config.friction.boost_duration_ms) { return false; }
        Motor3508_ResetSpeedPID();
        shoot_control_state.friction.state = SHOOT_FRIC_RECOVERY;
        shoot_control_state.friction.state_start_ms = now;
    }
    if (shoot_control_state.friction.state == SHOOT_FRIC_RECOVERY) {
        if ((uint32_t)(now - shoot_control_state.friction.state_start_ms) <
            shoot_config.friction.recovery_wait_ms) { return false; }
        shoot_control_state.friction.state = SHOOT_FRIC_NORMAL;
    }
    if (!Shoot_FricConfigValid() ||
        (uint32_t)(now - shoot_control_state.friction.state_start_ms) < shoot_config.friction.startup_grace_ms) {
        shoot_control_state.friction.block_timing[0] = false;
        shoot_control_state.friction.block_timing[1] = false;
        return true;
    }
    for (i = 0; i < 2; ++i) {
        bool blocked = Motor3508_GetFeedback((uint8_t)(i + 1), &feedback) &&
            Shoot_AbsInt32(feedback.speed_rpm) < shoot_config.friction.block_speed_rpm &&
            Shoot_AbsInt32(feedback.current_raw) >= shoot_config.friction.block_current_raw;
        if (!blocked) { shoot_control_state.friction.block_timing[i] = false; continue; }
        if (!shoot_control_state.friction.block_timing[i]) {
            shoot_control_state.friction.block_timing[i] = true;
            shoot_control_state.friction.block_start_ms[i] = now;
        }
        if ((uint32_t)(now - shoot_control_state.friction.block_start_ms[i]) <
            shoot_config.friction.block_confirm_ms) { continue; }
        shoot_control_state.friction.block_timing[0] = false;
        shoot_control_state.friction.block_timing[1] = false;
        if (shoot_control_state.friction.attempts >= shoot_config.friction.recovery_max_attempts) {
            shoot_control_state.friction.state = SHOOT_FRIC_FAULT;
        } else {
            shoot_control_state.friction.attempts++;
            shoot_control_state.friction.stuck_count++;
            shoot_control_state.friction.state = SHOOT_FRIC_BOOST;
            shoot_control_state.friction.state_start_ms = now;
            Motor3508_ResetSpeedPID();
        }
        return false;
    }
    return true;
}

// 双轮方向、误差和波动同时合格，才开始累计供弹就绪时间。
static bool Shoot_FricAtSpeed(void)
{
    uint32_t now = HAL_GetTick();
    bool stable = true;
    int32_t target = shoot_control_state.speed.target_speed_rpm;
    float tolerance = fmaxf(shoot_config.friction.ready.tolerance_rpm,
        fabsf((float)target) * shoot_config.friction.ready.relative_tolerance);
    if (!shoot_config.friction.ready.enabled) { shoot_control_state.ready.ready = true; return true; }
    if (!(tolerance >= 0.0f && isfinite(tolerance)) ||
        !(shoot_config.friction.ready.tolerance_rpm >= 0.0f && isfinite(shoot_config.friction.ready.tolerance_rpm)) ||
        !(shoot_config.friction.ready.relative_tolerance >= 0.0f && isfinite(shoot_config.friction.ready.relative_tolerance)) ||
        !(shoot_config.friction.ready.ripple_rpm >= 0.0f && isfinite(shoot_config.friction.ready.ripple_rpm)) || target <= 0)
    { stable = false; }
    for (uint8_t i = 0; i < 2U; ++i)
    {
        Motor3508_Feedback_t feedback;
        float direction = i == 0U ? motor3508_config.left_direction : motor3508_config.right_direction;
        if (!Motor3508_GetFeedback((uint8_t)(i + 1U), &feedback)) { stable = false; continue; }
        shoot_control_state.ready.error_rpm[i] = (float)feedback.speed_rpm - direction * (float)target;
        if (fabsf(shoot_control_state.ready.error_rpm[i]) > tolerance ||
            (shoot_control_state.ready.sampled && fabsf((float)feedback.speed_rpm -
             (float)shoot_control_state.ready.previous_rpm[i]) > shoot_config.friction.ready.ripple_rpm))
        { stable = false; }
        shoot_control_state.ready.previous_rpm[i] = feedback.speed_rpm;
    }
    if (shoot_control_state.ready.target_rpm != target) { stable = false; }
    shoot_control_state.ready.target_rpm = target;
    shoot_control_state.ready.sampled = true;
    if (!stable) { shoot_control_state.ready.timing = false; shoot_control_state.ready.ready = false; }
    else if (!shoot_control_state.ready.timing)
    { shoot_control_state.ready.timing = true; shoot_control_state.ready.stable_start_ms = now; }
    shoot_control_state.ready.ready = stable && shoot_control_state.ready.timing &&
        (uint32_t)(now - shoot_control_state.ready.stable_start_ms) >= shoot_config.friction.ready.stable_ms;
    return shoot_control_state.ready.ready;
}

static void Shoot_FricOutput(void)
{
    if (shoot_control_state.friction.state == SHOOT_FRIC_BOOST && !Shoot_FricConfigValid()) {
        shoot_control_state.friction.state = SHOOT_FRIC_FAULT;
    }
    if (shoot_control_state.friction.state == SHOOT_FRIC_FAULT) {
        (void)Motor3508_Stop();
    } else if (shoot_control_state.friction.state == SHOOT_FRIC_BOOST) {
        int32_t current = shoot_config.friction.boost_current_raw;
        // 左右轮按各自出弹方向给电流，仍经过驱动电流限幅。
        (void)Motor3508_SendCurrent(
            (int16_t)(current * motor3508_config.left_direction),
            (int16_t)(current * motor3508_config.right_direction));
    } else {
        (void)Motor3508_SpeedControl((int16_t)shoot_control_state.speed.target_speed_rpm);
    }
}

static int64_t Shoot_AbsInt64(int64_t value)
{
    return value < 0 ? -value : value;
}

// 将固定单圈位置换成最近一圈的累计目标，避免多圈倒转。
static int64_t Shoot_DialHoldTarget(const DialMotor_Feedback_t *feedback)
{
    int32_t delta = (int32_t)shoot_config.dial.hold_encoder - feedback->encoder;
    if (delta > 32767) { delta -= 65536; }
    else if (delta < -32768) { delta += 65536; }
    return feedback->encoder_total + delta;
}

static bool Shoot_DialArrived(const DialMotor_Feedback_t *feedback)
{
    return Shoot_AbsInt64(shoot_control_state.dial.target -
                          feedback->encoder_total) <=
           shoot_config.dial.arrived_error_counts;
}

static bool Shoot_DialBlockCheck(const DialMotor_Feedback_t *feedback,
                                 bool moving)
{
    if (shoot_config.dial.block_confirm_ticks == 0U ||
        shoot_config.dial.block_speed_threshold_dps <= 0 ||
        shoot_config.dial.block_current_threshold < 0)
    {
        shoot_control_state.recovery.block_tick = 0U;
        return false;
    }
    bool blocked = moving &&
        Shoot_AbsInt32((int32_t)feedback->speed_dps) <
            shoot_config.dial.block_speed_threshold_dps &&
        Shoot_AbsInt32((int32_t)feedback->current_raw) >
            shoot_config.dial.block_current_threshold;

    if (blocked)
    {
        if (shoot_control_state.recovery.block_tick <
            shoot_config.dial.block_confirm_ticks)
        {
            shoot_control_state.recovery.block_tick++;
        }
    }
    else
    {
        shoot_control_state.recovery.block_tick = 0U;
    }
    return shoot_control_state.recovery.block_tick >=
           shoot_config.dial.block_confirm_ticks;
}

// 只累计没有向目标靠近的时间，慢速前进和判稳暂停分别处理。
static void Shoot_DialProgressRefresh(const DialMotor_Feedback_t *feedback,
                                       uint32_t now)
{
    shoot_control_state.dial.progress_start_ms = now;
    shoot_control_state.dial.progress_error_counts =
        Shoot_AbsInt64(shoot_control_state.dial.target - feedback->encoder_total);
}

static bool Shoot_DialMotionTimedOut(const DialMotor_Feedback_t *feedback,
                                      uint32_t now, uint32_t timeout_ms)
{
    int64_t error = Shoot_AbsInt64(shoot_control_state.dial.target - feedback->encoder_total);
    if (shoot_config.dial.progress_min_counts <= 0 || timeout_ms == 0U)
    { return true; }
    if (shoot_control_state.dial.progress_error_counts - error >=
        shoot_config.dial.progress_min_counts)
    { Shoot_DialProgressRefresh(feedback, now); }
    return (uint32_t)(now - shoot_control_state.dial.progress_start_ms) >= timeout_ms;
}

// 暂停供弹时锁住当前位置，保留原发射目标和解堵次数。
static void Shoot_DialPause(void)
{
    DialMotor_Feedback_t feedback;
    shoot_control_state.recovery.block_tick = 0U;
    if (DialMotor_GetFeedback(&feedback))
    {
        Shoot_DialProgressRefresh(&feedback, HAL_GetTick());
        if (!shoot_control_state.ready.paused)
        {
            shoot_control_state.ready.pause_target = feedback.encoder_total;
            shoot_control_state.ready.paused = true;
        }
    }
    if (shoot_control_state.ready.paused)
    { (void)DialMotor_PositionControl(shoot_control_state.ready.pause_target); }
}

// 清掉旧位置和待发请求，零电流失败时由后续周期重发。
static void Shoot_DialLatchFault(void)
{
    shoot_control_state.recovery.fault = true;
    shoot_control_state.recovery.rearm_ready = false;
    shoot_control_state.recovery.block_tick = 0U;
    shoot_control_state.remote.single_pending = false;
    shoot_control_state.ready.paused = false;
    shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
    shoot_control_state.dial.target_synced = false;
    shoot_control_state.dial.single_reserved = 0U;
    shoot_control_state.dial.single_completed = 0U;
    shoot_control_state.stop.holding = false;
    shoot_control_state.stop.stopped = false;
    continuous_tracking = false;
    DialMotor_ResetControl();
    Shoot_DialIdleHold();
}

static void Shoot_DialMotionTimeout(ShootDialReason reason)
{
    shoot_control_state.recovery.reason = reason;
    shoot_control_state.count.motion_timeout++;
    Shoot_DialLatchFault();
}

static void Shoot_DialStartReverse(const DialMotor_Feedback_t *feedback,
                                    bool continuous, ShootDialReason reason)
{
    int64_t error = shoot_control_state.dial.target - feedback->encoder_total;

    shoot_control_state.recovery.reason = reason;
    if (reason == SHOOT_DIAL_REASON_BLOCK)
    {
        shoot_control_state.count.stuck++;
        if (!continuous && shoot_control_state.recovery.single_blocks < UINT32_MAX)
        { shoot_control_state.recovery.single_blocks++; }
    }
    else { shoot_control_state.count.motion_timeout++; }
    if (shoot_control_state.recovery.attempts >= shoot_config.dial.recovery_max_attempts)
    { Shoot_DialLatchFault(); return; }
    if (!(shoot_config.dial.stuck_reverse_speed_dps > 0.0f && isfinite(shoot_config.dial.stuck_reverse_speed_dps)) ||
        !(shoot_config.dial.stuck_reverse_angle_deg > 0.0f && isfinite(shoot_config.dial.stuck_reverse_angle_deg)) ||
        shoot_config.dial.stuck_reverse_timeout_ms == 0U)
    {
        shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_REVERSE_CONFIG;
        Shoot_DialLatchFault(); return;
    }
    shoot_control_state.recovery.attempts++;
    shoot_control_state.recovery.reverse_start = feedback->encoder_total;
    shoot_control_state.recovery.reverse_travel_deg = 0.0f;

    shoot_control_state.recovery.continuous = continuous;
    // 保存原供弹目标；连发退让后直接恢复速度闭环。
    shoot_control_state.recovery.feed_target = continuous ?
        feedback->encoder_total : shoot_control_state.dial.target;
    shoot_control_state.recovery.motion_direction = continuous ?
        (int8_t)shoot_config.dial.feed_direction : (error < 0 ? -1 : 1);
    shoot_control_state.dial.state = SHOOT_DIAL_STUCK_REVERSE;
    shoot_control_state.dial.state_start_ms = HAL_GetTick();
    shoot_control_state.recovery.block_tick = 0U;
    DialMotor_ResetControl();
}

static void Shoot_DialEnterStuckRecovery(
    const DialMotor_Feedback_t *feedback, bool continuous)
{
    Shoot_DialStartReverse(feedback, continuous, SHOOT_DIAL_REASON_BLOCK);
}

// 已通过的供弹相位只计一次，解堵后重新经过时不重复累计。
static void Shoot_DialSingleProgress(const DialMotor_Feedback_t *feedback)
{
    int64_t progress = shoot_config.dial.feed_direction *
        (feedback->encoder_total - shoot_control_state.dial.single_origin);
    uint32_t completed;
    if (progress < -shoot_config.dial.arrived_error_counts) { return; }
    progress = (progress + shoot_config.dial.arrived_error_counts) / SHOOT_DIAL_ONE_BULLET_COUNTS;
    completed = progress >= shoot_control_state.dial.single_reserved ?
        shoot_control_state.dial.single_reserved : (uint32_t)progress;
    if (completed > shoot_control_state.dial.single_completed)
    {
        shoot_control_state.count.single += completed - shoot_control_state.dial.single_completed;
        shoot_control_state.dial.single_completed = completed;
    }
}

// 追加目标不清 PID，也不重置无进度计时，避免连续输入掩盖真实堵转。
static void Shoot_DialAppendSingles(void)
{
    uint32_t appended = 0U;
    while (shoot_control_state.keyboard.single_active && shoot_control_state.keyboard.single_pending > 0U &&
           appended < SHOOT_SINGLE_APPEND_PER_CYCLE)
    {
        if (!Shoot_HeatCanStart() || shoot_control_state.dial.single_reserved == UINT32_MAX)
        {
            shoot_control_state.keyboard.single_pending = 0U;
            break;
        }
        Shoot_HeatReserve();
        shoot_control_state.keyboard.single_pending--;
        shoot_control_state.dial.single_reserved++;
        shoot_control_state.dial.target += shoot_config.dial.feed_direction * SHOOT_DIAL_ONE_BULLET_COUNTS;
        shoot_control_state.recovery.feed_target = shoot_control_state.dial.target;
        shoot_control_state.dial.progress_error_counts += SHOOT_DIAL_ONE_BULLET_COUNTS;
        appended++;
    }
}

static void Shoot_DialUpdate(bool single_rising, bool continuous)
{
    DialMotor_Feedback_t feedback;
    uint32_t now = HAL_GetTick();

    if (shoot_config.dial.feed_direction != 1 && shoot_config.dial.feed_direction != -1)
    { Shoot_DialIdleHold(); return; }
    if (!DialMotor_GetFeedback(&feedback) || !feedback.initialized)
    {
        return;
    }

    if (!shoot_control_state.dial.target_synced)
    {
        shoot_control_state.dial.target = Shoot_DialHoldTarget(&feedback);
        shoot_control_state.recovery.feed_target =
            shoot_control_state.dial.target;
        shoot_control_state.dial.target_synced = true;
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        DialMotor_ResetControl();
    }

    if (shoot_control_state.dial.state == SHOOT_DIAL_FEED ||
        shoot_control_state.dial.state == SHOOT_DIAL_STUCK_RELOAD ||
        (shoot_control_state.dial.state == SHOOT_DIAL_STUCK_REVERSE && !shoot_control_state.recovery.continuous))
    {
        Shoot_DialSingleProgress(&feedback);
        if (!continuous) { Shoot_DialAppendSingles(); }
    }

    if (continuous && !continuous_tracking &&
        (Shoot_HeatCanStart() || shoot_control_state.dial.state == SHOOT_DIAL_FEED ||
         shoot_control_state.dial.state == SHOOT_DIAL_STUCK_REVERSE ||
         shoot_control_state.dial.state == SHOOT_DIAL_STUCK_RELOAD))
    {
        bool carry_single = shoot_control_state.dial.state == SHOOT_DIAL_FEED ||
            shoot_control_state.dial.state == SHOOT_DIAL_STUCK_REVERSE ||
            shoot_control_state.dial.state == SHOOT_DIAL_STUCK_RELOAD;
        continuous_tracking = true;
        if (carry_single)
        {
            // 单发转连发沿用本发相位和热量，退让中的动作先完成限时反转。
            continuous_origin = shoot_control_state.dial.single_origin;
            continuous_reserved = shoot_control_state.dial.single_reserved;
            shoot_control_state.recovery.continuous = true;
        }
        else
        {
            continuous_origin = Shoot_DialHoldTarget(&feedback);
            if (shoot_config.dial.feed_direction * (feedback.encoder_total - continuous_origin) <
                -shoot_config.dial.arrived_error_counts)
            { continuous_origin -= shoot_config.dial.feed_direction * SHOOT_DIAL_ONE_BULLET_COUNTS; }
            Shoot_HeatReserve();
            continuous_reserved = 1U;
        }
        if (shoot_control_state.dial.state != SHOOT_DIAL_STUCK_REVERSE)
        { shoot_control_state.dial.state = SHOOT_DIAL_CONTINUOUS; }
    }

    switch (shoot_control_state.dial.state)
    {
    case SHOOT_DIAL_IDLE:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (single_rising && Shoot_HeatCanStart())
        {
            Shoot_HeatReserve();
            shoot_control_state.dial.single_origin = shoot_control_state.dial.target;
            shoot_control_state.dial.single_reserved = 1U;
            shoot_control_state.dial.single_completed = 0U;
            // 本车拨盘逆时针为上弹方向，对应编码器正方向。
            shoot_control_state.dial.target +=
                shoot_config.dial.feed_direction *
                SHOOT_DIAL_ONE_BULLET_COUNTS;
            shoot_control_state.recovery.feed_target =
                shoot_control_state.dial.target;
            Shoot_DialAppendSingles();
            shoot_control_state.dial.state = SHOOT_DIAL_FEED;
            shoot_control_state.dial.state_start_ms = now;
            Shoot_DialProgressRefresh(&feedback, now);
            shoot_control_state.recovery.block_tick = 0U;
            DialMotor_ResetControl();
        }
        break;

    case SHOOT_DIAL_FEED:
        if (Shoot_DialBlockCheck(&feedback,
                !Shoot_DialArrived(&feedback)))
        {
            Shoot_DialEnterStuckRecovery(&feedback, false);
        }
        else if (Shoot_DialArrived(&feedback))
        {
            (void)DialMotor_PositionControl(shoot_control_state.dial.target);
            shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
            shoot_control_state.recovery.block_tick = 0U;
            shoot_control_state.recovery.single_blocks = 0U;
            shoot_control_state.recovery.attempts = 0U;
        }
        else if (Shoot_DialMotionTimedOut(&feedback, now, shoot_config.dial.single_move_timeout_ms))
        { Shoot_DialMotionTimeout(SHOOT_DIAL_REASON_FEED_TIMEOUT); }
        else { (void)DialMotor_PositionControl(shoot_control_state.dial.target); }
        break;

    case SHOOT_DIAL_CONTINUOUS:
    {
        ShootHeatConfig config = shoot_heat_config;
        int64_t progress = shoot_config.dial.feed_direction *
            (feedback.encoder_total - continuous_origin);
        if (!Shoot_HeatCanStart() && progress >=
            (int64_t)continuous_reserved * SHOOT_DIAL_ONE_BULLET_COUNTS -
            shoot_config.dial.arrived_error_counts)
        {
            // 沿用拨盘到位容差，避免末发在微小误差内继续判堵转。
            shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
            shoot_control_state.recovery.block_tick = 0U;
            shoot_control_state.stop.holding = false;
            continuous_tracking = false;
            Shoot_DialIdleHold();
            return;
        }
        // 只有越过新的正向供弹圈才预留下一发，退让与重装不重复计热。
        while (progress >= (int64_t)continuous_reserved * SHOOT_DIAL_ONE_BULLET_COUNTS)
        {
            if (!Shoot_HeatCanStart())
            {
                shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
                shoot_control_state.stop.holding = false;
                continuous_tracking = false;
                Shoot_DialIdleHold();
                return;
            }
            Shoot_HeatReserve();
            ++continuous_reserved;
        }
        if (Shoot_HeatCanStart())
        {
            (void)DialMotor_SpeedControl((float)shoot_config.dial.feed_direction *
                360.0f * ShootHeat_GetRate(&config));
        }
        else
        {
            // 完成已计热的末发，仍走速度环；到供弹终点再切保持。
            (void)DialMotor_SpeedControl((float)shoot_config.dial.feed_direction *
                360.0f * config.low_rate_hz);
        }
        if (Shoot_DialBlockCheck(&feedback, true))
        {
            Shoot_DialEnterStuckRecovery(&feedback, true);
        }
        break;
    }

    case SHOOT_DIAL_STUCK_REVERSE:
    {
        uint32_t elapsed = now - shoot_control_state.dial.state_start_ms;
        bool retreated;
        bool blocked = false;
        shoot_control_state.recovery.reverse_travel_deg =
            (float)(shoot_control_state.recovery.motion_direction *
            (shoot_control_state.recovery.reverse_start - feedback.encoder_total)) *
            (360.0f / (float)SHOOT_DIAL_ONE_BULLET_COUNTS);
        retreated = shoot_control_state.recovery.reverse_travel_deg >= shoot_config.dial.stuck_reverse_angle_deg;
        if (elapsed >= shoot_config.dial.stuck_reverse_grace_ms)
        { blocked = Shoot_DialBlockCheck(&feedback, !retreated); }
        else { shoot_control_state.recovery.block_tick = 0U; }
        if (!(shoot_config.dial.stuck_reverse_speed_dps > 0.0f &&
              isfinite(shoot_config.dial.stuck_reverse_speed_dps)) ||
            !(shoot_config.dial.stuck_reverse_angle_deg > 0.0f &&
              isfinite(shoot_config.dial.stuck_reverse_angle_deg)) ||
            shoot_config.dial.stuck_reverse_timeout_ms == 0U)
        {
            shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_REVERSE_CONFIG;
            Shoot_DialLatchFault();
        }
        else if (blocked)
        {
            shoot_control_state.count.reverse_stuck++;
            shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_REVERSE_BLOCK;
            Shoot_DialLatchFault();
        }
        else if (retreated || elapsed >= shoot_config.dial.stuck_reverse_timeout_ms)
        {
            // 连发直接恢复速度环，单发继续原目标；解堵沿用原热量预留。
            DialMotor_ResetControl();
            if (DialMotor_SpeedControl(0.0f) != HAL_OK)
            {
                shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_REVERSE_STOP;
                Shoot_DialLatchFault();
                break;
            }
            if (!retreated)
            {
                shoot_control_state.count.reverse_timeout++;
                shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_REVERSE_TIMEOUT;
                Shoot_DialLatchFault();
                break;
            }
            shoot_control_state.dial.target =
                shoot_control_state.recovery.feed_target;
            shoot_control_state.dial.state = shoot_control_state.recovery.continuous ?
                SHOOT_DIAL_CONTINUOUS : SHOOT_DIAL_STUCK_RELOAD;
            shoot_control_state.dial.state_start_ms = now;
            Shoot_DialProgressRefresh(&feedback, now);
            shoot_control_state.recovery.block_tick = 0U;
        }
        else
        { (void)DialMotor_SpeedControl(-shoot_control_state.recovery.motion_direction *
              shoot_config.dial.stuck_reverse_speed_dps); }
        break;
    }

    case SHOOT_DIAL_STUCK_RELOAD:
        if (Shoot_DialArrived(&feedback))
        {
            (void)DialMotor_PositionControl(shoot_control_state.dial.target);
            shoot_control_state.dial.state =
                shoot_control_state.recovery.continuous ?
                SHOOT_DIAL_CONTINUOUS : SHOOT_DIAL_IDLE;
            shoot_control_state.recovery.block_tick = 0U;
            if (shoot_control_state.recovery.continuous)
            {
                DialMotor_ResetControl();
            }
            else
            {
                shoot_control_state.recovery.single_blocks = 0U;
                shoot_control_state.recovery.attempts = 0U;
            }
        }
        else if (Shoot_DialBlockCheck(&feedback, true))
        { Shoot_DialEnterStuckRecovery(&feedback, shoot_control_state.recovery.continuous); }
        else if (Shoot_DialMotionTimedOut(&feedback, now, shoot_config.dial.stuck_reload_timeout_ms))
        { Shoot_DialStartReverse(&feedback, false, SHOOT_DIAL_REASON_RELOAD_TIMEOUT); }
        else { (void)DialMotor_PositionControl(shoot_control_state.dial.target); }
        break;

    default:
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.dial.target_synced = false;
        DialMotor_ResetControl();
        break;
    }
}

void ShootControl_SetIdleHoldEnabled(bool enabled)
{
    idle_hold_enabled = enabled;
}

static void Shoot_DialIdleHold(void)
{
    DialMotor_Feedback_t feedback;
    bool hold = !shoot_control_state.recovery.fault &&
        idle_hold_enabled && DialMotor_OnlineCheck() &&
        DialMotor_GetFeedback(&feedback) && feedback.initialized;
    if (hold) {
        if (!shoot_control_state.stop.holding) {
            // 上线和停止供弹都回固定相位，保持期间不随反馈改目标。
            shoot_control_state.dial.target = Shoot_DialHoldTarget(&feedback);
            DialMotor_ResetControl();
            shoot_control_state.stop.holding = true;
            shoot_control_state.dial.target_synced = true;
        }
        shoot_control_state.stop.stopped = false;
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        return;
    }
    // 失联或解堵失败时给零电流，故障期间不恢复位置保持。
    if (shoot_control_state.stop.holding) { DialMotor_ResetControl(); }
    shoot_control_state.stop.holding = false;
    shoot_control_state.dial.target_synced = false;
    if ((!shoot_control_state.stop.stopped || !DialMotor_OnlineCheck() ||
         (uint32_t)(HAL_GetTick() - shoot_control_state.stop.last_stop_ms) >=
            shoot_config.dial.safe_stop_retry_ms) &&
        DialMotor_SetTorqueCurrent(0) == HAL_OK) {
        shoot_control_state.stop.stopped = true;
        shoot_control_state.stop.last_stop_ms = HAL_GetTick();
    }
}

// 初始化热量和弹速调节，清空供弹、堵转及输入状态，启用待机保持。
void ShootControl_Init(void)
{
    shoot_control_state.ready = (ShootReadyState_t){0};
    shoot_control_state.recovery.attempts = 0U;
    shoot_control_state.recovery.fault = false;
    shoot_control_state.recovery.rearm_ready = false;
    shoot_control_state.recovery.single_blocks = 0U;
    shoot_control_state.recovery.reason = SHOOT_DIAL_REASON_NONE;
    shoot_control_state.remote.single_pending = false;
    ShootHeat_Init(HAL_GetTick());
    ShootSpeed_Init(&shoot_control_state.speed);
    continuous_tracking = false;
    idle_hold_enabled = true;
    shoot_control_state.stop.holding = false;
    Shoot_FricReset();
    shoot_control_state.friction.stuck_count = 0;
    shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
    shoot_control_state.count.single = 0U;
    shoot_control_state.count.stuck = 0U;
    shoot_control_state.count.motion_timeout = 0U;
    shoot_control_state.count.reverse_timeout = 0U;
    shoot_control_state.count.reverse_stuck = 0U;
    shoot_control_state.recovery.reverse_start = 0;
    shoot_control_state.recovery.reverse_travel_deg = 0.0f;
    shoot_control_state.dial.progress_start_ms = HAL_GetTick();
    shoot_control_state.dial.progress_error_counts = 0;
    shoot_control_state.dial.single_origin = 0;
    shoot_control_state.dial.single_reserved = 0U;
    shoot_control_state.dial.single_completed = 0U;
    shoot_control_state.recovery.block_tick = 0U;
    shoot_control_state.dial.target = 0;
    shoot_control_state.recovery.feed_target = 0;
    shoot_control_state.recovery.motion_direction =
        (int8_t)shoot_config.dial.feed_direction;
    shoot_control_state.dial.target_synced = false;
    shoot_control_state.recovery.continuous = false;
    shoot_control_state.stop.stopped = false;
    shoot_control_state.stop.last_stop_ms = 0U;
    shoot_control_state.remote.last_right_up = false;
    shoot_control_state.remote.last_mode = REMOTE_SHOOT_OFF;
    shoot_control_state.remote.continuous_requested = false;
    shoot_control_state.keyboard.single_seen = 0U;
    shoot_control_state.keyboard.single_pending = false;
    shoot_control_state.keyboard.single_dropped = 0U;
    shoot_control_state.keyboard.single_active = false;
    shoot_control_state.keyboard.single_started = false;
    Motor3508_ResetSpeedPID();
    DialMotor_ResetControl();
}

void ShootControl_Update(RemoteShoot_t mode, bool right_up)
{
    bool single_rising;
    bool friction_ready;
    bool trigger_released;
    bool new_continuous;
    ShootHeatConfig heat_config = shoot_heat_config;
    Shoot_HeatUpdate();

    if (mode != REMOTE_SHOOT_OFF && mode != REMOTE_SHOOT_READY &&
        mode != REMOTE_SHOOT_SINGLE && mode != REMOTE_SHOOT_CONTINUOUS)
    {
        mode = REMOTE_SHOOT_OFF;
    }
    single_rising = right_up && !shoot_control_state.remote.last_right_up &&
                    mode == REMOTE_SHOOT_SINGLE;
    new_continuous = mode == REMOTE_SHOOT_CONTINUOUS &&
        !shoot_control_state.remote.continuous_requested;
    shoot_control_state.remote.continuous_requested = mode == REMOTE_SHOOT_CONTINUOUS;
    trigger_released = !right_up &&
        (mode == REMOTE_SHOOT_OFF || mode == REMOTE_SHOOT_READY);

    if (!shoot_heat_state.config_valid ||
        (shoot_heat_state.referee_valid && !shoot_heat_state.output_allowed) ||
        (heat_config.enabled && !shoot_heat_state.referee_valid && !heat_config.allow_offline))
    { if (mode != REMOTE_SHOOT_OFF) { mode = REMOTE_SHOOT_READY; } }

    friction_ready = Shoot_FricPrepare(mode != REMOTE_SHOOT_OFF);
    {
        Communication_ShotSnapshot_t shot;
        ShootSpeedFeedback feedback;
        ShootSpeedConfig config = shoot_config.friction.adaptive;
        uint32_t now = HAL_GetTick();
        bool received = Communication_GetShotSnapshot(&shot);
        bool ready = friction_ready && shoot_control_state.friction.enabled &&
            shoot_control_state.friction.state == SHOOT_FRIC_NORMAL &&
            (uint32_t)(now - shoot_control_state.friction.state_start_ms) >= shoot_config.friction.startup_grace_ms;
        if (received) {
            feedback.actual_m_s = shot.speed_m_s;
            feedback.limit_m_s = shot.speed_limit_m_s;
            feedback.sequence = shot.sequence;
            feedback.valid = shot.valid;
            feedback.limit_valid = shot.speed_limit_valid;
            feedback.received_ms = shot.last_rx_ms;
            feedback.sample_ms = shot.sample_ms;
        }
        ShootSpeed_Update(&shoot_control_state.speed, &config, shoot_config.friction.target_speed_rpm,
            motor3508_config.max_speed_rpm, received ? &feedback : NULL, ready, now);
    }
    // 恢复期间停拨盘，避免继续供弹；群组电流仍在拨盘命令之后发送。
    if (shoot_control_state.recovery.fault && trigger_released)
    { shoot_control_state.recovery.rearm_ready = true; }

    // 新单发边沿或释放后的连发重新接管；保持原输入不能自动解锁。
    if (shoot_control_state.recovery.fault &&
        ((mode == REMOTE_SHOOT_SINGLE && single_rising) ||
         (mode == REMOTE_SHOOT_CONTINUOUS && shoot_control_state.recovery.rearm_ready)))
    {
        shoot_control_state.recovery.fault = false;
        shoot_control_state.recovery.rearm_ready = false;
        shoot_control_state.recovery.single_blocks = 0U;
        shoot_control_state.recovery.attempts = 0U;
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.dial.target_synced = false;
        DialMotor_ResetControl();
    }

    // 新连发操作建立重试预算，摩擦轮掉速产生的内部待发不能刷新预算。
    if (new_continuous && !shoot_control_state.recovery.fault &&
        shoot_control_state.dial.state != SHOOT_DIAL_STUCK_REVERSE &&
        shoot_control_state.dial.state != SHOOT_DIAL_STUCK_RELOAD)
    {
        shoot_control_state.recovery.attempts = 0U;
        shoot_control_state.recovery.single_blocks = 0U;
    }

    if (mode == REMOTE_SHOOT_OFF || mode == REMOTE_SHOOT_READY)
    {
        shoot_control_state.remote.single_pending = false;
        shoot_control_state.dial.single_reserved = 0U;
        shoot_control_state.dial.single_completed = 0U;
        if (trigger_released && !shoot_control_state.recovery.fault &&
            shoot_control_state.recovery.continuous)
        {
            shoot_control_state.recovery.attempts = 0U;
            shoot_control_state.recovery.single_blocks = 0U;
        }
        shoot_control_state.ready.paused = false;
        if (mode == REMOTE_SHOOT_OFF)
        { shoot_control_state.ready = (ShootReadyState_t){0}; }
        else { (void)Shoot_FricAtSpeed(); }
        continuous_tracking = false;
        // 拨盘帧先入队，关摩擦轮时也不能让群组帧占掉最后一个邮箱。
        Shoot_DialIdleHold();
        // 保险、断控和许可撤销都主动停轮，拨盘帧仍先入队。
        if (mode == REMOTE_SHOOT_OFF) { (void)Motor3508_BrakeStop(); }
        // 待发时先给拨盘保活，再发摩擦轮 0x200，避免邮箱被占满。
        if (mode == REMOTE_SHOOT_READY)
        { Shoot_FricOutput(); }
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.remote.last_right_up = right_up;
        shoot_control_state.remote.last_mode = mode;
        return;
    }

    shoot_control_state.stop.holding = false;
    shoot_control_state.stop.stopped = false;
    if (mode != shoot_control_state.remote.last_mode &&
        shoot_control_state.dial.state != SHOOT_DIAL_STUCK_REVERSE &&
        shoot_control_state.dial.state != SHOOT_DIAL_STUCK_RELOAD &&
        !(mode == REMOTE_SHOOT_CONTINUOUS && shoot_control_state.dial.state == SHOOT_DIAL_FEED))
    {
        shoot_control_state.remote.single_pending = false;
        continuous_tracking = false;
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.dial.target_synced = false;
        DialMotor_ResetControl();
    }
    if (single_rising) { shoot_control_state.remote.single_pending = true; }
    if (!Shoot_HeatCanStart()) { shoot_control_state.remote.single_pending = false; }
    if (shoot_control_state.recovery.fault)
    { Shoot_DialIdleHold(); }
    else if (shoot_control_state.dial.state == SHOOT_DIAL_STUCK_REVERSE)
    {
        // 退让独立计时，摩擦轮判稳只拦截正向供弹。
        Shoot_DialUpdate(false, mode == REMOTE_SHOOT_CONTINUOUS);
    }
    else if (!friction_ready)
    { Shoot_DialPause(); }
    else if (!Shoot_FricAtSpeed())
    { Shoot_DialPause(); }
    else
    {
        if (shoot_control_state.ready.paused) { DialMotor_ResetControl(); shoot_control_state.ready.paused = false; }
        Shoot_DialUpdate(shoot_control_state.remote.single_pending, mode == REMOTE_SHOOT_CONTINUOUS);
        if (shoot_control_state.dial.state != SHOOT_DIAL_IDLE) { shoot_control_state.remote.single_pending = false; }
    }
    // 4005 控制帧优先入队，随后再发送摩擦轮群组帧。
    Shoot_FricOutput();
    shoot_control_state.remote.last_right_up = right_up;
    shoot_control_state.remote.last_mode = mode;
}

void ShootControl_ResetKeyboard(uint32_t single_request_count)
{
    shoot_control_state.keyboard.single_seen = single_request_count;
    shoot_control_state.keyboard.single_pending = 0U;
    shoot_control_state.keyboard.single_active = false;
    shoot_control_state.keyboard.single_started = false;
}

void ShootControl_UpdateKeyboard(RemoteShoot_t mode,
                                 uint32_t single_request_count)
{
    if (mode != REMOTE_SHOOT_READY && mode != REMOTE_SHOOT_CONTINUOUS)
    {
        ShootControl_ResetKeyboard(single_request_count);
        ShootControl_Update(REMOTE_SHOOT_OFF, false);
        return;
    }

    // 连发优先接管，排队单发并入本次连发，松开后不补旧请求。
    if (mode == REMOTE_SHOOT_CONTINUOUS)
    {
        ShootControl_ResetKeyboard(single_request_count);
        ShootControl_Update(mode, false);
        return;
    }

    if (single_request_count != shoot_control_state.keyboard.single_seen)
    {
        uint32_t added = single_request_count - shoot_control_state.keyboard.single_seen;
        uint32_t limit = shoot_config.single_queue_limit;
        uint32_t pending = shoot_control_state.keyboard.single_pending;
        uint32_t active = shoot_control_state.keyboard.single_active ?
            (shoot_control_state.keyboard.single_started ?
             shoot_control_state.dial.single_reserved - shoot_control_state.dial.single_completed : 1U) : 0U;
        shoot_control_state.keyboard.single_seen = single_request_count;
        if (limit < UINT32_MAX) { limit++; }
        limit = active >= limit ? 0U : limit - active;
        if (added < 0x80000000U)
        {
            if (pending > limit)
            {
                shoot_control_state.keyboard.single_dropped += pending - limit;
                pending = limit;
            }
            if (added > limit - pending)
            {
                shoot_control_state.keyboard.single_dropped += added - (limit - pending);
                added = limit - pending;
            }
            shoot_control_state.keyboard.single_pending = pending + added;
        }
        else { shoot_control_state.keyboard.single_pending = 0U; }
    }

    if (shoot_control_state.keyboard.single_active ||
        (shoot_control_state.keyboard.single_pending && mode == REMOTE_SHOOT_READY))
    {
        if (!shoot_control_state.keyboard.single_active)
        {
            shoot_control_state.keyboard.single_active = true;
            shoot_control_state.keyboard.single_started = false;
            shoot_control_state.keyboard.single_pending--;
            shoot_control_state.remote.last_right_up = false;
        }

        // 松键后的单发不能立即退回 READY，否则拨盘刚起动就会被 Stop。
        ShootControl_Update(REMOTE_SHOOT_SINGLE,
                            !shoot_control_state.keyboard.single_started);
        if (shoot_control_state.recovery.fault)
        {
            shoot_control_state.keyboard.single_active = false;
            shoot_control_state.keyboard.single_pending = 0U;
            shoot_control_state.keyboard.single_started = false;
            return;
        }
        if (!Shoot_HeatCanStart())
        { shoot_control_state.keyboard.single_pending = 0U; }
        if (shoot_control_state.dial.state != SHOOT_DIAL_IDLE)
        { shoot_control_state.keyboard.single_started = true; }
        else if (shoot_control_state.keyboard.single_started)
        { shoot_control_state.keyboard.single_active = false; }
        else
        {
            if (!Shoot_HeatCanStart())
            {
                shoot_control_state.keyboard.single_active = false;
                shoot_control_state.keyboard.single_pending = 0U;
                return;
            }
            // 反馈暂未就绪时允许下周期重新尝试单发上升沿。
            shoot_control_state.remote.last_right_up = false;
        }
        return;
    }

    ShootControl_Update(mode, false);
}
