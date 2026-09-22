#include "shoot_control.h"
#include "dial_motor.h"
#include "motor3508.h"
#include "parameter.h"
#include "stm32f4xx_hal.h"

volatile ShootDialState_t shoot_dial_state = SHOOT_DIAL_IDLE;
volatile uint32_t shoot_single_count;
volatile uint32_t shoot_dial_stuck_count;

static uint32_t dial_block_tick;
static uint32_t dial_state_start_ms;
static int64_t dial_target;
static int64_t dial_feed_target;
static int8_t dial_motion_direction;
static bool dial_target_synced;
static bool last_single_shot_command;

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

static bool Shoot_DialBlockCheck(const DialMotor_Feedback_t *feedback)
{
    int64_t error = dial_target - feedback->encoder_total;
    bool moving = Shoot_AbsInt64(error) > SHOOT_DIAL_ARRIVED_ERROR_COUNTS;
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
    const DialMotor_Feedback_t *feedback)
{
    int64_t error = dial_target - feedback->encoder_total;

    dial_feed_target = dial_target;
    dial_motion_direction = error < 0 ? -1 : 1;
    dial_target = feedback->encoder_total -
                  (int64_t)dial_motion_direction *
                  SHOOT_DIAL_ONE_BULLET_COUNTS;
    shoot_dial_state = SHOOT_DIAL_STUCK_REVERSE;
    dial_state_start_ms = HAL_GetTick();
    dial_block_tick = 0U;
    shoot_dial_stuck_count++;
    DialMotor_ResetControl();
}

static void Shoot_DialUpdate(bool single_rising)
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
        shoot_dial_state = SHOOT_DIAL_IDLE;
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
        if (Shoot_DialBlockCheck(&feedback))
        {
            Shoot_DialEnterStuckRecovery(&feedback);
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
            shoot_dial_state = SHOOT_DIAL_IDLE;
            dial_block_tick = 0U;
            shoot_single_count++;
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
    last_single_shot_command = false;
    Motor3508_ResetSpeedPID();
    DialMotor_ResetControl();
}

void ShootControl_Update(bool control_enabled, bool single_shot_command)
{
    bool single_rising = single_shot_command &&
                         !last_single_shot_command;

    if (!control_enabled)
    {
        (void)Motor3508_Stop();
        DialMotor_ResetControl();
        (void)DialMotor_SetTorqueCurrent(0);
        shoot_dial_state = SHOOT_DIAL_IDLE;
        dial_block_tick = 0U;
        dial_target_synced = false;
        last_single_shot_command = false;
        return;
    }

    (void)Motor3508_SpeedControl(SHOOT_FRIC_TARGET_SPEED_RPM);
    Shoot_DialUpdate(single_rising);
    last_single_shot_command = single_shot_command;
}
