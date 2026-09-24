#include "shoot_control.h"
#include "dial_motor.h"
#include "motor3508.h"
#include "parameter.h"
#include "stm32f4xx_hal.h"

volatile ShootDialState_t shoot_dial_state = SHOOT_DIAL_IDLE; /* 当前拨盘状态机状态。 */
volatile uint32_t shoot_single_count;      /* 已完成的累计单发次数。 */
volatile uint32_t shoot_dial_stuck_count; /* 累计触发拨盘堵转恢复的次数。 */

static uint32_t dial_block_tick;    /* 连续满足堵转条件的控制周期数。 */
static uint32_t dial_state_start_ms;/* 当前拨盘状态的进入时间。 */
static int64_t dial_target;         /* 当前拨盘累计编码器目标。 */
static int64_t dial_feed_target;    /* 堵转退让前保存的原供弹目标。 */
static int8_t dial_motion_direction;/* 堵转前运动方向，取值为 -1 或 1。 */
static bool dial_target_synced;     /* 目标是否已与当前实际位置同步。 */
static bool recovery_continuous;    /* 堵转恢复前是否处于连发模式。 */
static bool dial_stopped;           /* 拨盘停止命令是否已成功入队。 */
static uint32_t dial_last_stop_ms;  /* 最近一次停止命令入队时间。 */
static bool last_right_up;          /* 上周期右拨杆上档状态。 */
static RemoteShoot_t last_mode;     /* 上周期发射模式，用于切换时重建目标。 */
static uint32_t keyboard_single_seen; /* 发射任务已读取的短按事件序号。 */
static bool keyboard_single_pending; /* 当前一发进行中时，最多暂存下一次短按。 */
static bool keyboard_single_active; /* 单发已触发，需保持位控至结束。 */
static bool keyboard_single_started; /* 拨盘已进入供弹/堵转恢复状态。 */

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
    return Shoot_AbsInt64(dial_target - feedback->encoder_total) <=
           SHOOT_DIAL_ARRIVED_ERROR_COUNTS;
}

static bool Shoot_DialBlockCheck(const DialMotor_Feedback_t *feedback,
                                 bool moving)
{
    bool blocked = moving &&
        Shoot_AbsInt32((int32_t)feedback->speed_dps) <
            SHOOT_DIAL_BLOCK_SPEED_THRESHOLD_DPS &&
        Shoot_AbsInt32((int32_t)feedback->current_raw) >
            SHOOT_DIAL_BLOCK_CURRENT_THRESHOLD;

    if (blocked)
    {
        if (dial_block_tick < SHOOT_DIAL_BLOCK_CONFIRM_TICKS)
        {
            dial_block_tick++;
        }
    }
    else
    {
        dial_block_tick = 0U;
    }
    return dial_block_tick >= SHOOT_DIAL_BLOCK_CONFIRM_TICKS;
}

static void Shoot_DialEnterStuckRecovery(
    const DialMotor_Feedback_t *feedback, bool continuous)
{
    int64_t error = dial_target - feedback->encoder_total;

    recovery_continuous = continuous;
    /* 连发无固定终点：退让后返回堵转前的位置，再恢复速度闭环。 */
    dial_feed_target = continuous ? feedback->encoder_total : dial_target;
    dial_motion_direction = continuous ? (int8_t)SHOOT_DIAL_FEED_DIRECTION :
                                         (error < 0 ? -1 : 1);
    dial_target = feedback->encoder_total -
                  (int64_t)dial_motion_direction *
                  SHOOT_DIAL_ONE_BULLET_COUNTS;
    shoot_dial_state = SHOOT_DIAL_STUCK_REVERSE;
    dial_state_start_ms = HAL_GetTick();
    dial_block_tick = 0U;
    shoot_dial_stuck_count++;
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

    if (!dial_target_synced)
    {
        dial_target = feedback.encoder_total;
        dial_feed_target = dial_target;
        dial_target_synced = true;
        shoot_dial_state = continuous ? SHOOT_DIAL_CONTINUOUS :
                                         SHOOT_DIAL_IDLE;
        DialMotor_ResetControl();
    }

    switch (shoot_dial_state)
    {
    case SHOOT_DIAL_IDLE:
        (void)DialMotor_PositionControl(dial_target);
        if (single_rising)
        {
            /* 本车拨盘逆时针为上弹方向，对应编码器正方向。 */
            dial_target += SHOOT_DIAL_FEED_DIRECTION *
                           SHOOT_DIAL_ONE_BULLET_COUNTS;
            dial_feed_target = dial_target;
            shoot_dial_state = SHOOT_DIAL_FEED;
            dial_state_start_ms = now;
            dial_block_tick = 0U;
            DialMotor_ResetControl();
        }
        break;

    case SHOOT_DIAL_FEED:
        (void)DialMotor_PositionControl(dial_target);
        if (Shoot_DialBlockCheck(&feedback,
                Shoot_AbsInt64(dial_target - feedback.encoder_total) >
                    SHOOT_DIAL_ARRIVED_ERROR_COUNTS))
        {
            Shoot_DialEnterStuckRecovery(&feedback, false);
        }
        else if (Shoot_DialArrived(&feedback) ||
                 (uint32_t)(now - dial_state_start_ms) >=
                     SHOOT_DIAL_SINGLE_MOVE_TIMEOUT_MS)
        {
            shoot_dial_state = SHOOT_DIAL_IDLE;
            dial_block_tick = 0U;
            shoot_single_count++;
        }
        break;

    case SHOOT_DIAL_CONTINUOUS:
        (void)DialMotor_SpeedControl(
            (float)SHOOT_DIAL_FEED_DIRECTION * 360.0f *
            SHOOT_DIAL_CONTINUOUS_ROUNDS_PER_S);
        if (Shoot_DialBlockCheck(&feedback, true))
        {
            Shoot_DialEnterStuckRecovery(&feedback, true);
        }
        break;

    case SHOOT_DIAL_STUCK_REVERSE:
        (void)DialMotor_PositionControl(dial_target);
        if (Shoot_DialArrived(&feedback) ||
            (uint32_t)(now - dial_state_start_ms) >=
                SHOOT_DIAL_STUCK_REVERSE_TIMEOUT_MS)
        {
            /* 退让完成后继续追原来的累计上弹目标。 */
            dial_target = dial_feed_target;
            shoot_dial_state = SHOOT_DIAL_STUCK_RELOAD;
            dial_state_start_ms = now;
            DialMotor_ResetControl();
        }
        break;

    case SHOOT_DIAL_STUCK_RELOAD:
        (void)DialMotor_PositionControl(dial_target);
        if (Shoot_DialArrived(&feedback) ||
            (uint32_t)(now - dial_state_start_ms) >=
                SHOOT_DIAL_STUCK_RELOAD_TIMEOUT_MS)
        {
            shoot_dial_state = recovery_continuous ?
                SHOOT_DIAL_CONTINUOUS : SHOOT_DIAL_IDLE;
            dial_block_tick = 0U;
            if (recovery_continuous)
            {
                DialMotor_ResetControl();
            }
            else
            {
                shoot_single_count++;
            }
        }
        break;

    default:
        shoot_dial_state = SHOOT_DIAL_IDLE;
        dial_block_tick = 0U;
        dial_target_synced = false;
        DialMotor_ResetControl();
        break;
    }
}

void ShootControl_Init(void)
{
    shoot_dial_state = SHOOT_DIAL_IDLE;
    shoot_single_count = 0U;
    shoot_dial_stuck_count = 0U;
    dial_block_tick = 0U;
    dial_target = 0;
    dial_feed_target = 0;
    dial_motion_direction = (int8_t)SHOOT_DIAL_FEED_DIRECTION;
    dial_target_synced = false;
    recovery_continuous = false;
    dial_stopped = false;
    dial_last_stop_ms = 0U;
    last_right_up = false;
    last_mode = REMOTE_SHOOT_OFF;
    keyboard_single_seen = 0U;
    keyboard_single_pending = false;
    keyboard_single_active = false;
    keyboard_single_started = false;
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
    single_rising = right_up && !last_right_up &&
                    mode == REMOTE_SHOOT_SINGLE;

    if (mode == REMOTE_SHOOT_OFF || mode == REMOTE_SHOOT_READY)
    {
        if (mode == REMOTE_SHOOT_OFF)
        {
            (void)Motor3508_Stop();
        }
        else
        {
            (void)Motor3508_SpeedControl(SHOOT_FRIC_TARGET_SPEED_RPM);
        }
        /* 保险及只转摩擦轮模式停止拨盘，周期重发以覆盖掉线恢复。 */
        if ((!dial_stopped ||
             (uint32_t)(HAL_GetTick() - dial_last_stop_ms) >=
                 SHOOT_DIAL_SAFE_STOP_RETRY_MS) &&
            DialMotor_Stop() == HAL_OK)
        {
            dial_stopped = true;
            dial_last_stop_ms = HAL_GetTick();
        }
        shoot_dial_state = SHOOT_DIAL_IDLE;
        dial_block_tick = 0U;
        dial_target_synced = false;
        last_right_up = right_up;
        last_mode = mode;
        return;
    }

    (void)Motor3508_SpeedControl(SHOOT_FRIC_TARGET_SPEED_RPM);
    if (dial_stopped)
    {
        if (DialMotor_Run() != HAL_OK)
        {
            /* 运行命令未入队时保留上升沿，下一周期继续尝试单发。 */
            return;
        }
        dial_stopped = false;
    }
    if (mode != last_mode)
    {
        shoot_dial_state = SHOOT_DIAL_IDLE;
        dial_block_tick = 0U;
        dial_target_synced = false;
        DialMotor_ResetControl();
    }
    Shoot_DialUpdate(single_rising, mode == REMOTE_SHOOT_CONTINUOUS);
    last_right_up = right_up;
    last_mode = mode;
}

void ShootControl_ResetKeyboard(uint32_t single_request_count)
{
    keyboard_single_seen = single_request_count;
    keyboard_single_pending = false;
    keyboard_single_active = false;
    keyboard_single_started = false;
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

    if (single_request_count != keyboard_single_seen)
    {
        keyboard_single_seen = single_request_count;
        keyboard_single_pending = true;
    }

    if (keyboard_single_active ||
        (keyboard_single_pending && mode == REMOTE_SHOOT_READY))
    {
        if (!keyboard_single_active)
        {
            keyboard_single_active = true;
            keyboard_single_started = false;
            keyboard_single_pending = false;
        }

        /* 松键后的单发不能立即退回 READY，否则拨盘刚起动就会被 Stop。 */
        ShootControl_Update(REMOTE_SHOOT_SINGLE,
                            !keyboard_single_started);
        if (shoot_dial_state != SHOOT_DIAL_IDLE)
        { keyboard_single_started = true; }
        else if (keyboard_single_started)
        { keyboard_single_active = false; }
        else
        {
            /* 反馈暂未就绪时允许下周期重新尝试单发上升沿。 */
            last_right_up = false;
        }
        return;
    }

    if (mode == REMOTE_SHOOT_CONTINUOUS)
    { keyboard_single_pending = false; }
    ShootControl_Update(mode, false);
}
