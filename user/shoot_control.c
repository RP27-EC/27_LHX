#include "shoot_control.h"
#include "dial_motor.h"
#include "motor3508.h"
#include "application_config.h"
#include "stm32f4xx_hal.h"

// 集中保存拨盘、堵转、停机及键鼠单发状态，便于调试观察。
volatile ShootControlState_t shoot_control_state = {
    .dial.state = SHOOT_DIAL_IDLE,
};

static int32_t Shoot_AbsInt32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int64_t Shoot_AbsInt64(int64_t value)
{
    return value < 0 ? -value : value;
}

static bool Shoot_DialArrived(const DialMotor_Feedback_t *feedback)
{
    return Shoot_AbsInt64(shoot_control_state.dial.target -
                          feedback->encoder_total) <=
           shoot_config.dial_arrived_error_counts;
}

static bool Shoot_DialBlockCheck(const DialMotor_Feedback_t *feedback,
                                 bool moving)
{
    bool blocked = moving &&
        Shoot_AbsInt32((int32_t)feedback->speed_dps) <
            shoot_config.dial_block_speed_threshold_dps &&
        Shoot_AbsInt32((int32_t)feedback->current_raw) >
            shoot_config.dial_block_current_threshold;

    if (blocked)
    {
        if (shoot_control_state.recovery.block_tick <
            shoot_config.dial_block_confirm_ticks)
        {
            shoot_control_state.recovery.block_tick++;
        }
    }
    else
    {
        shoot_control_state.recovery.block_tick = 0U;
    }
    return shoot_control_state.recovery.block_tick >=
           shoot_config.dial_block_confirm_ticks;
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
        (int8_t)shoot_config.dial_feed_direction : (error < 0 ? -1 : 1);
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

    if (!DialMotor_GetFeedback(&feedback))
    {
        return;
    }

    if (!shoot_control_state.dial.target_synced)
    {
        shoot_control_state.dial.target = feedback.encoder_total;
        shoot_control_state.recovery.feed_target =
            shoot_control_state.dial.target;
        shoot_control_state.dial.target_synced = true;
        shoot_control_state.dial.state = continuous ?
            SHOOT_DIAL_CONTINUOUS : SHOOT_DIAL_IDLE;
        DialMotor_ResetControl();
    }

    switch (shoot_control_state.dial.state)
    {
    case SHOOT_DIAL_IDLE:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (single_rising)
        {
            // 本车拨盘逆时针为上弹方向，对应编码器正方向。
            shoot_control_state.dial.target +=
                shoot_config.dial_feed_direction *
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
                Shoot_AbsInt64(shoot_control_state.dial.target - feedback.encoder_total) >
                    shoot_config.dial_arrived_error_counts))
        {
            Shoot_DialEnterStuckRecovery(&feedback, false);
        }
        else if (Shoot_DialArrived(&feedback) ||
                 (uint32_t)(now - shoot_control_state.dial.state_start_ms) >=
                     shoot_config.dial_single_move_timeout_ms)
        {
            shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
            shoot_control_state.recovery.block_tick = 0U;
            shoot_control_state.count.single++;
        }
        break;

    case SHOOT_DIAL_CONTINUOUS:
        (void)DialMotor_SpeedControl(
            (float)shoot_config.dial_feed_direction * 360.0f *
            shoot_config.dial_continuous_rounds_per_s);
        if (Shoot_DialBlockCheck(&feedback, true))
        {
            Shoot_DialEnterStuckRecovery(&feedback, true);
        }
        break;

    case SHOOT_DIAL_STUCK_REVERSE:
        (void)DialMotor_PositionControl(shoot_control_state.dial.target);
        if (Shoot_DialArrived(&feedback) ||
            (uint32_t)(now - shoot_control_state.dial.state_start_ms) >=
                shoot_config.dial_stuck_reverse_timeout_ms)
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
                shoot_config.dial_stuck_reload_timeout_ms)
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

void ShootControl_Init(void)
{
    shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
    shoot_control_state.count.single = 0U;
    shoot_control_state.count.stuck = 0U;
    shoot_control_state.recovery.block_tick = 0U;
    shoot_control_state.dial.target = 0;
    shoot_control_state.recovery.feed_target = 0;
    shoot_control_state.recovery.motion_direction =
        (int8_t)shoot_config.dial_feed_direction;
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

    if (mode != REMOTE_SHOOT_OFF && mode != REMOTE_SHOOT_READY &&
        mode != REMOTE_SHOOT_SINGLE && mode != REMOTE_SHOOT_CONTINUOUS)
    {
        mode = REMOTE_SHOOT_OFF;
    }
    single_rising = right_up && !shoot_control_state.remote.last_right_up &&
                    mode == REMOTE_SHOOT_SINGLE;

    if (mode == REMOTE_SHOOT_OFF || mode == REMOTE_SHOOT_READY)
    {
        if (mode == REMOTE_SHOOT_OFF)
        {
            (void)Motor3508_Stop();
        }
        else
        {
            (void)Motor3508_SpeedControl(shoot_config.fric_target_speed_rpm);
        }
        // 安全态持续发送零电流，与正常控制共用 A1 回报链路。
        DialMotor_ResetControl();
        if ((!shoot_control_state.stop.stopped ||
             (uint32_t)(HAL_GetTick() - shoot_control_state.stop.last_stop_ms) >=
                 shoot_config.dial_safe_stop_retry_ms) &&
            DialMotor_SetTorqueCurrent(0) == HAL_OK)
        {
            shoot_control_state.stop.stopped = true;
            shoot_control_state.stop.last_stop_ms = HAL_GetTick();
        }
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.dial.target_synced = false;
        shoot_control_state.remote.last_right_up = right_up;
        shoot_control_state.remote.last_mode = mode;
        return;
    }

    (void)Motor3508_SpeedControl(shoot_config.fric_target_speed_rpm);
    shoot_control_state.stop.stopped = false;
    if (mode != shoot_control_state.remote.last_mode)
    {
        shoot_control_state.dial.state = SHOOT_DIAL_IDLE;
        shoot_control_state.recovery.block_tick = 0U;
        shoot_control_state.dial.target_synced = false;
        DialMotor_ResetControl();
    }
    Shoot_DialUpdate(single_rising, mode == REMOTE_SHOOT_CONTINUOUS);
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
            // 反馈暂未就绪时允许下周期重新尝试单发上升沿。
            shoot_control_state.remote.last_right_up = false;
        }
        return;
    }

    if (mode == REMOTE_SHOOT_CONTINUOUS)
    { shoot_control_state.keyboard.single_pending = false; }
    ShootControl_Update(mode, false);
}
