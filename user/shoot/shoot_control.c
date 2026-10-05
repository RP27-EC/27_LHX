#include "shoot_control.h"
#include "dial_motor.h"
#include "motor3508.h"
#include "application_config.h"
#include "peripheral_config.h"
#include "stm32f4xx_hal.h"
#include "communication.h"

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
        (void)Motor3508_SpeedControl(shoot_config.friction.target_speed_rpm);
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

static void Shoot_DialEnterStuckRecovery(
    const DialMotor_Feedback_t *feedback, bool continuous)
{
    int64_t error = shoot_control_state.dial.target - feedback->encoder_total;

    shoot_control_state.recovery.continuous = continuous;
    // 连发无固定终点：退让后返回堵转前的位置，再恢复速度闭环。
    shoot_control_state.recovery.feed_target = continuous ?
        feedback->encoder_total : shoot_control_state.dial.target;
    shoot_control_state.recovery.motion_direction = continuous ?
        (int8_t)shoot_config.dial.feed_direction : (error < 0 ? -1 : 1);
    shoot_control_state.dial.target = feedback->encoder_total -
        (int64_t)shoot_control_state.recovery.motion_direction *
        SHOOT_DIAL_ONE_BULLET_COUNTS;
    shoot_control_state.dial.state = SHOOT_DIAL_STUCK_REVERSE;
    shoot_control_state.dial.state_start_ms = HAL_GetTick();
    shoot_control_state.recovery.block_tick = 0U;
    shoot_control_state.count.stuck++;
    DialMotor_ResetControl();
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

    if (continuous && !continuous_tracking && Shoot_HeatCanStart())
    {
        continuous_tracking = true;
        // 以供弹固定相位建立圈数基准，末发终点同时作为保持相位。
        continuous_origin = Shoot_DialHoldTarget(&feedback);
        if (shoot_config.dial.feed_direction * (feedback.encoder_total - continuous_origin) <
            -shoot_config.dial.arrived_error_counts)
        { continuous_origin -= shoot_config.dial.feed_direction * SHOOT_DIAL_ONE_BULLET_COUNTS; }
        continuous_reserved = 1U;
        Shoot_HeatReserve();
        shoot_control_state.dial.state = SHOOT_DIAL_CONTINUOUS;
    }

    switch (shoot_control_state.dial.state)
    {
    case SHOOT_DIAL_IDLE:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (single_rising && Shoot_HeatCanStart())
        {
            Shoot_HeatReserve();
            // 本车拨盘逆时针为上弹方向，对应编码器正方向。
            shoot_control_state.dial.target +=
                shoot_config.dial.feed_direction *
                SHOOT_DIAL_ONE_BULLET_COUNTS;
            shoot_control_state.recovery.feed_target =
                shoot_control_state.dial.target;
            shoot_control_state.dial.state = SHOOT_DIAL_FEED;
            shoot_control_state.dial.state_start_ms = now;
            shoot_control_state.recovery.block_tick = 0U;
            DialMotor_ResetControl();
        }
        break;

    case SHOOT_DIAL_FEED:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (Shoot_DialBlockCheck(&feedback,
                !Shoot_DialArrived(&feedback)))
        {
            Shoot_DialEnterStuckRecovery(&feedback, false);
        }
        else if (Shoot_DialArrived(&feedback) ||
                 (uint32_t)(now - shoot_control_state.dial.state_start_ms) >=
                     shoot_config.dial.single_move_timeout_ms)
        {
            shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
            shoot_control_state.recovery.block_tick = 0U;
            shoot_control_state.count.single++;
        }
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
            // 本发已计热，完成这一圈后停在供弹终点。
            shoot_control_state.dial.target = continuous_origin +
                shoot_config.dial.feed_direction * (int64_t)continuous_reserved *
                SHOOT_DIAL_ONE_BULLET_COUNTS;
            (void)DialMotor_PositionControlLimited(shoot_control_state.dial.target,
                360.0f * config.low_rate_hz);
        }
        if (Shoot_DialBlockCheck(&feedback, true))
        {
            Shoot_DialEnterStuckRecovery(&feedback, true);
        }
        break;
    }

    case SHOOT_DIAL_STUCK_REVERSE:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (Shoot_DialArrived(&feedback) ||
            (uint32_t)(now - shoot_control_state.dial.state_start_ms) >=
                shoot_config.dial.stuck_reverse_timeout_ms)
        {
            // 退让完成后继续追原来的累计上弹目标。
            shoot_control_state.dial.target =
                shoot_control_state.recovery.feed_target;
            shoot_control_state.dial.state = SHOOT_DIAL_STUCK_RELOAD;
            shoot_control_state.dial.state_start_ms = now;
            DialMotor_ResetControl();
        }
        break;

    case SHOOT_DIAL_STUCK_RELOAD:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (Shoot_DialArrived(&feedback) ||
            (uint32_t)(now - shoot_control_state.dial.state_start_ms) >=
                shoot_config.dial.stuck_reload_timeout_ms)
        {
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
                shoot_control_state.count.single++;
            }
        }
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
    bool hold = idle_hold_enabled && DialMotor_OnlineCheck() &&
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
    // 只有失联时发零电流，不沿用旧位置目标。
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

// 初始化本地热量，清空供弹、堵转恢复和输入边沿状态，启用待机位置保持。
void ShootControl_Init(void)
{
    ShootHeat_Init(HAL_GetTick());
    continuous_tracking = false;
    idle_hold_enabled = true;
    shoot_control_state.stop.holding = false;
    Shoot_FricReset();
    shoot_control_state.friction.stuck_count = 0;
    shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
    shoot_control_state.count.single = 0U;
    shoot_control_state.count.stuck = 0U;
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
    shoot_control_state.keyboard.single_seen = 0U;
    shoot_control_state.keyboard.single_pending = false;
    shoot_control_state.keyboard.single_active = false;
    shoot_control_state.keyboard.single_started = false;
    Motor3508_ResetSpeedPID();
    DialMotor_ResetControl();
}

void ShootControl_Update(RemoteShoot_t mode, bool right_up)
{
    bool single_rising;
    bool friction_ready;
    ShootHeatConfig heat_config = shoot_heat_config;
    Shoot_HeatUpdate();

    if (mode != REMOTE_SHOOT_OFF && mode != REMOTE_SHOOT_READY &&
        mode != REMOTE_SHOOT_SINGLE && mode != REMOTE_SHOOT_CONTINUOUS)
    {
        mode = REMOTE_SHOOT_OFF;
    }
    single_rising = right_up && !shoot_control_state.remote.last_right_up &&
                    mode == REMOTE_SHOOT_SINGLE;

    if (!shoot_heat_state.config_valid ||
        (shoot_heat_state.referee_valid && !shoot_heat_state.output_allowed) ||
        (heat_config.enabled && !shoot_heat_state.referee_valid && !heat_config.allow_offline))
    { if (mode != REMOTE_SHOOT_OFF) { mode = REMOTE_SHOOT_READY; } }

    friction_ready = Shoot_FricPrepare(mode != REMOTE_SHOOT_OFF);
    // 恢复期间停拨盘，避免继续供弹；群组电流仍在拨盘命令之后发送。
    if (mode != REMOTE_SHOOT_OFF && !friction_ready) { mode = REMOTE_SHOOT_READY; }

    if (mode == REMOTE_SHOOT_OFF || mode == REMOTE_SHOOT_READY)
    {
        continuous_tracking = false;
        // 拨盘帧先入队，关摩擦轮时也不能让群组帧占掉最后一个邮箱。
        Shoot_DialIdleHold();
        if (mode == REMOTE_SHOOT_OFF) { (void)Motor3508_Stop(); }
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
    if (mode != shoot_control_state.remote.last_mode)
    {
        continuous_tracking = false;
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.dial.target_synced = false;
        DialMotor_ResetControl();
    }
    Shoot_DialUpdate(single_rising, mode == REMOTE_SHOOT_CONTINUOUS);
    // 4005 控制帧优先入队，随后再发送摩擦轮群组帧。
    Shoot_FricOutput();
    shoot_control_state.remote.last_right_up = right_up;
    shoot_control_state.remote.last_mode = mode;
}

void ShootControl_ResetKeyboard(uint32_t single_request_count)
{
    shoot_control_state.keyboard.single_seen = single_request_count;
    shoot_control_state.keyboard.single_pending = false;
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

    if (single_request_count != shoot_control_state.keyboard.single_seen)
    {
        shoot_control_state.keyboard.single_seen = single_request_count;
        shoot_control_state.keyboard.single_pending = true;
    }

    if (shoot_control_state.keyboard.single_active ||
        (shoot_control_state.keyboard.single_pending && mode == REMOTE_SHOOT_READY))
    {
        if (!shoot_control_state.keyboard.single_active)
        {
            shoot_control_state.keyboard.single_active = true;
            shoot_control_state.keyboard.single_started = false;
            shoot_control_state.keyboard.single_pending = false;
        }

        // 松键后的单发不能立即退回 READY，否则拨盘刚起动就会被 Stop。
        ShootControl_Update(REMOTE_SHOOT_SINGLE,
                            !shoot_control_state.keyboard.single_started);
        if (shoot_control_state.dial.state != SHOOT_DIAL_IDLE)
        { shoot_control_state.keyboard.single_started = true; }
        else if (shoot_control_state.keyboard.single_started)
        { shoot_control_state.keyboard.single_active = false; }
        else
        {
            if (!Shoot_HeatCanStart())
            {
                shoot_control_state.keyboard.single_active = false;
                shoot_control_state.keyboard.single_pending = false;
                return;
            }
            // 反馈暂未就绪时允许下周期重新尝试单发上升沿。
            shoot_control_state.remote.last_right_up = false;
        }
        return;
    }

    if (mode == REMOTE_SHOOT_CONTINUOUS)
    { shoot_control_state.keyboard.single_pending = false; }
    ShootControl_Update(mode, false);
}
