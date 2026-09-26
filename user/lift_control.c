#include "lift_control.h"
#include "cloud_terrace.h"
#include "communication.h"
#include "motor2006.h"
#include "parameter.h"
#include "stm32f4xx_hal.h"

#define LIFT_ENCODER_COUNTS_PER_TURN 8192.0f

volatile LiftControl_State_t lift_control_state;
volatile LiftControl_WaitReason_t lift_wait_reason;
volatile bool lift_calibrated;
volatile int32_t lift_top_encoder_total;
volatile int32_t lift_bottom_encoder_total;

static LiftControl_StallSnapshot_t stall_snapshot;
static bool right_switch_seen;
static uint8_t previous_right_switch;
static LiftControl_State_t requested_direction;
static bool calibration_active;
static bool calibration_backoff;
static bool calibration_armed;
static bool calibration_fault_latched;
static bool calibration_drive_started;
static bool lift_hold_target_valid;
static int32_t lift_hold_target_encoder_total;
static uint32_t calibration_start_ms;
static bool settle_timing;
static uint32_t settle_start_ms;
static bool stall_timing;
static uint32_t stall_start_ms;
static bool progress_timing;
static uint32_t progress_start_ms;
static int32_t progress_start_counts;
static bool progress_high_current_all;
static bool motor_stopped;
static uint32_t last_stop_ms;
static bool chassis_hold_request;
static uint8_t chassis_hold_sequence;
static bool chassis_hold_tx_seen;
static uint32_t chassis_hold_tx_ms;
static uint32_t chassis_hold_start_ms;
static bool offline_release_timing;
static uint32_t offline_release_start_ms;

static int32_t LiftControl_Abs(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t LiftControl_DownSign(void)
{
    return LIFT_DOWN_DIRECTION > 0.0f ? 1 : -1;
}

static void LiftControl_ResetStallCheck(void)
{
    stall_timing = false;
    progress_timing = false;
}

static void LiftControl_Stop(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t retry_ms = (lift_control_state == LIFT_STALLED ||
                         lift_control_state == LIFT_AT_LIMIT) ?
                        LIFT_FAULT_STOP_RETRY_MS : LIFT_STOP_RETRY_MS;

    if ((!motor_stopped ||
         (uint32_t)(now - last_stop_ms) >= retry_ms) &&
        Motor2006_Stop() == HAL_OK)
    {
        motor_stopped = true;
        last_stop_ms = now;
    }
}

static void LiftControl_RunSpeed(float speed_rad_s)
{
    motor_stopped = false;
    offline_release_timing = false;
    (void)Motor2006_SpeedControl(speed_rad_s);
}

static void LiftControl_SendChassisHold(bool hold, uint32_t now)
{
    if (hold && !chassis_hold_request)
    {
        chassis_hold_request = true;
        chassis_hold_sequence++;
        chassis_hold_tx_seen = false;
        chassis_hold_start_ms = now;
    }
    else if (!hold && chassis_hold_request)
    {
        chassis_hold_request = false;
        chassis_hold_tx_seen = false;
    }
    if ((!chassis_hold_tx_seen ||
         (uint32_t)(now - chassis_hold_tx_ms) >= LIFT_LOCK_TX_PERIOD_MS) &&
        Communication_CAN_SendLiftLock(chassis_hold_request,
                                       chassis_hold_sequence) == HAL_OK)
    {
        chassis_hold_tx_seen = true;
        chassis_hold_tx_ms = now;
    }
}

static bool LiftControl_ChassisReady(uint32_t now)
{
    LiftControl_SendChassisHold(true, now);
    if (!chassis_hold_tx_seen ||
        (uint32_t)(now - chassis_hold_start_ms) <
            LIFT_CHASSIS_LOCK_SETTLE_MS ||
        !Communication_CAN_ChassisWheelsStopped(chassis_hold_start_ms))
    {
        lift_wait_reason = LIFT_WAIT_CHASSIS_SPEED;
        return false;
    }
    lift_wait_reason = LIFT_WAIT_NONE;
    return true;
}

static void LiftControl_StopAndRelease(uint32_t now)
{
    Motor2006_Feedback_t feedback;
    bool moving;

    LiftControl_Stop();
    if (Motor2006_OnlineCheck() && Motor2006_GetFeedback(&feedback))
    {
        offline_release_timing = false;
        moving = LiftControl_Abs((int32_t)feedback.speed_rpm) >
                 LIFT_CHASSIS_RELEASE_RPM;
    }
    else
    {
        if (!offline_release_timing)
        {
            offline_release_timing = true;
            offline_release_start_ms = now;
        }
        moving = (uint32_t)(now - offline_release_start_ms) <
                 LIFT_OFFLINE_RELEASE_MS;
    }
    if (chassis_hold_request && (!motor_stopped || moving))
    { LiftControl_SendChassisHold(true, now); }
    else
    { LiftControl_SendChassisHold(false, now); }
}

static bool LiftControl_PowerOnLimit(const Motor2006_Feedback_t *feedback,
                                      LiftControl_State_t direction)
{
    int32_t distance = feedback->encoder_total * LiftControl_DownSign();
    int32_t limit = (int32_t)(LIFT_MAX_ROTOR_TURNS *
                              LIFT_ENCODER_COUNTS_PER_TURN);

    if (direction == LIFT_ASCENDING) { distance = -distance; }
    return limit - distance <= LIFT_LIMIT_STOP_MARGIN_COUNTS;
}

static void LiftControl_RecordStall(const Motor2006_Feedback_t *feedback,
                                    LiftControl_State_t direction,
                                    uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    stall_snapshot.direction = direction;
    stall_snapshot.encoder = feedback->encoder;
    stall_snapshot.encoder_total = feedback->encoder_total;
    stall_snapshot.rotor_turns = (float)feedback->encoder_total /
                                 LIFT_ENCODER_COUNTS_PER_TURN;
    stall_snapshot.time_ms = now;
    stall_snapshot.valid = true;
    __set_PRIMASK(primask);
}

/* 高电流低速或持续无位移均为堵转；只有持续高电流才可认作顶部接触。 */
static bool LiftControl_StallCheck(const Motor2006_Feedback_t *feedback,
                                   LiftControl_State_t direction,
                                   uint32_t now, bool *top_contact)
{
    int32_t current_threshold = direction == LIFT_ASCENDING ?
        LIFT_UP_STALL_CURRENT_RAW : LIFT_DOWN_STALL_CURRENT_RAW;
    int32_t speed_threshold = direction == LIFT_ASCENDING ?
        LIFT_UP_STALL_SPEED_RPM : LIFT_DOWN_STALL_SPEED_RPM;
    uint32_t duration = direction == LIFT_ASCENDING ?
        LIFT_UP_STALL_TIME_MS : LIFT_DOWN_STALL_TIME_MS;
    bool high_current = LiftControl_Abs((int32_t)feedback->current_raw) >=
                        current_threshold;
    bool low_speed = LiftControl_Abs((int32_t)feedback->speed_rpm) <=
                     speed_threshold;
    bool current_stall, no_progress;

    if (!progress_timing)
    {
        progress_timing = true;
        progress_start_ms = now;
        progress_start_counts = feedback->encoder_total;
        progress_high_current_all = high_current;
    }
    else if (!high_current)
    { progress_high_current_all = false; }

    if (!high_current || !low_speed) { stall_timing = false; }
    else if (!stall_timing)
    {
        stall_timing = true;
        stall_start_ms = now;
    }
    current_stall = stall_timing &&
        (uint32_t)(now - stall_start_ms) >= duration;
    no_progress = (uint32_t)(now - progress_start_ms) >= duration &&
        LiftControl_Abs(feedback->encoder_total - progress_start_counts) <
            LIFT_STALL_PROGRESS_COUNTS;
    if (current_stall || no_progress)
    {
        *top_contact = current_stall ||
                       (no_progress && progress_high_current_all);
        LiftControl_RecordStall(feedback, direction, now);
        LiftControl_ResetStallCheck();
        return true;
    }
    if ((uint32_t)(now - progress_start_ms) >= duration)
    {
        progress_start_ms = now;
        progress_start_counts = feedback->encoder_total;
        progress_high_current_all = high_current;
    }
    return false;
}

static bool LiftControl_PositionArrived(const Motor2006_Feedback_t *feedback,
                                        int32_t target)
{
    return LiftControl_Abs(target - feedback->encoder_total) <=
           LIFT_POSITION_TOLERANCE_COUNTS;
}

static LiftControl_State_t LiftControl_DirectionTo(int32_t error)
{
    return error * LiftControl_DownSign() > 0 ?
        LIFT_DESCENDING : LIFT_ASCENDING;
}

static void LiftControl_PositionDrive(const Motor2006_Feedback_t *feedback,
                                       int32_t target, float speed_limit)
{
    float speed = (float)(target - feedback->encoder_total) /
                  LIFT_ENCODER_COUNTS_PER_TURN *
                  LIFT_POSITION_KP_RAD_S_PER_TURN;

    if (speed > speed_limit) { speed = speed_limit; }
    else if (speed < -speed_limit) { speed = -speed_limit; }
    if (speed > 0.0f && speed < LIFT_POSITION_MIN_SPEED_RAD_S)
    { speed = LIFT_POSITION_MIN_SPEED_RAD_S; }
    else if (speed < 0.0f && speed > -LIFT_POSITION_MIN_SPEED_RAD_S)
    { speed = -LIFT_POSITION_MIN_SPEED_RAD_S; }
    LiftControl_RunSpeed(speed);
}

static void LiftControl_CalibrationFail(LiftControl_State_t reason)
{
    calibration_active = false;
    calibration_armed = false; /* 本次上线不自动重试。 */
    calibration_fault_latched = true;
    requested_direction = LIFT_STOPPED;
    LiftControl_ResetStallCheck();
    lift_control_state = reason;
    lift_wait_reason = LIFT_WAIT_FAULT;
    LiftControl_StopAndRelease(HAL_GetTick());
}

static void LiftControl_Calibrate(const Motor2006_Feedback_t *feedback,
                                  uint32_t now)
{
    bool top_contact = false;
    int32_t top, bottom;

    if (!calibration_backoff)
    {
        if ((calibration_drive_started &&
             (uint32_t)(now - calibration_start_ms) >=
                 LIFT_CALIBRATE_TIMEOUT_MS) ||
            LiftControl_PowerOnLimit(feedback, LIFT_ASCENDING))
        {
            LiftControl_CalibrationFail(LIFT_AT_LIMIT);
            return;
        }
        if (!LiftControl_ChassisReady(now))
        {
            lift_control_state = LIFT_CALIBRATING_UP;
            LiftControl_ResetStallCheck();
            LiftControl_Stop();
            return;
        }
        if (!calibration_drive_started)
        {
            calibration_drive_started = true;
            calibration_start_ms = now;
        }
        if (LiftControl_StallCheck(feedback, LIFT_ASCENDING, now,
                                   &top_contact))
        {
            if (!top_contact)
            {
                LiftControl_CalibrationFail(LIFT_STALLED);
                return;
            }
            top = feedback->encoder_total + LiftControl_DownSign() *
                (int32_t)(LIFT_TOP_BACKOFF_TURNS *
                          LIFT_ENCODER_COUNTS_PER_TURN);
            bottom = top + LiftControl_DownSign() *
                (int32_t)(LIFT_TRAVEL_TURNS *
                          LIFT_ENCODER_COUNTS_PER_TURN);
            if (LiftControl_Abs(top) >= (int32_t)(LIFT_MAX_ROTOR_TURNS *
                    LIFT_ENCODER_COUNTS_PER_TURN) ||
                LiftControl_Abs(bottom) >= (int32_t)(LIFT_MAX_ROTOR_TURNS *
                    LIFT_ENCODER_COUNTS_PER_TURN))
            {
                LiftControl_CalibrationFail(LIFT_AT_LIMIT);
                return;
            }
            lift_top_encoder_total = top;
            lift_bottom_encoder_total = bottom;
            calibration_backoff = true;
            calibration_start_ms = now;
            settle_timing = false;
            lift_control_state = LIFT_CALIBRATING_BACKOFF;
            LiftControl_Stop();
            return;
        }
        lift_control_state = LIFT_CALIBRATING_UP;
        LiftControl_RunSpeed(-LIFT_DOWN_DIRECTION *
                              LIFT_CALIBRATE_UP_SPEED_RAD_S);
        return;
    }

    if ((uint32_t)(now - calibration_start_ms) >=
        LIFT_BACKOFF_TIMEOUT_MS)
    {
        LiftControl_CalibrationFail(LIFT_STALLED);
        return;
    }
    if (LiftControl_PositionArrived(feedback, lift_top_encoder_total))
    {
        if (!LiftControl_ChassisReady(now))
        {
            LiftControl_ResetStallCheck();
            LiftControl_Stop();
            return;
        }
        LiftControl_ResetStallCheck();
        if (LiftControl_Abs((int32_t)feedback->speed_rpm) <=
            LIFT_POSITION_SETTLED_RPM)
        {
            if (!settle_timing)
            {
                settle_timing = true;
                settle_start_ms = now;
            }
            else if ((uint32_t)(now - settle_start_ms) >=
                     LIFT_CALIBRATION_SETTLE_MS)
            {
                calibration_active = false;
                lift_calibrated = true;
                lift_hold_target_encoder_total = lift_top_encoder_total;
                lift_hold_target_valid = true;
                right_switch_seen = false;
                requested_direction = LIFT_STOPPED;
                lift_control_state = LIFT_READY;
                LiftControl_StopAndRelease(now);
                return;
            }
        }
        else { settle_timing = false; }
        lift_control_state = LIFT_CALIBRATING_BACKOFF;
        LiftControl_RunSpeed(0.0f);
        return;
    }
    settle_timing = false;
    if (LiftControl_PowerOnLimit(feedback,
            LiftControl_DirectionTo(lift_top_encoder_total -
                                    feedback->encoder_total)))
    {
        LiftControl_CalibrationFail(LIFT_AT_LIMIT);
        return;
    }
    if (!LiftControl_ChassisReady(now))
    {
        LiftControl_ResetStallCheck();
        LiftControl_Stop();
        return;
    }
    if (LiftControl_StallCheck(feedback,
            LiftControl_DirectionTo(lift_top_encoder_total -
                                    feedback->encoder_total),
            now, &top_contact))
    {
        LiftControl_CalibrationFail(LIFT_STALLED);
        return;
    }
    lift_control_state = LIFT_CALIBRATING_BACKOFF;
    LiftControl_PositionDrive(feedback, lift_top_encoder_total,
                               LIFT_BACKOFF_SPEED_RAD_S);
}

void LiftControl_Init(void)
{
    lift_control_state = LIFT_STOPPED;
    lift_wait_reason = LIFT_WAIT_REMOTE;
    lift_calibrated = false;
    lift_top_encoder_total = 0;
    lift_bottom_encoder_total = 0;
    stall_snapshot.valid = false;
    right_switch_seen = false;
    previous_right_switch = 0U;
    requested_direction = LIFT_STOPPED;
    calibration_active = false;
    calibration_backoff = false;
    calibration_armed = true;
    calibration_fault_latched = false;
    calibration_drive_started = false;
    lift_hold_target_valid = false;
    lift_hold_target_encoder_total = 0;
    calibration_start_ms = 0U;
    settle_timing = false;
    LiftControl_ResetStallCheck();
    motor_stopped = false;
    last_stop_ms = 0U;
    chassis_hold_request = false;
    chassis_hold_sequence = 0U;
    chassis_hold_tx_seen = false;
    chassis_hold_tx_ms = 0U;
    chassis_hold_start_ms = 0U;
    offline_release_timing = false;
    offline_release_start_ms = 0U;
    LiftControl_StopAndRelease(HAL_GetTick());
}

bool LiftControl_GetStallSnapshot(LiftControl_StallSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) { return false; }
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = stall_snapshot;
    __set_PRIMASK(primask);
    return snapshot->valid;
}

void LiftControl_Update(const RemoteState_t *remote)
{
    Motor2006_Feedback_t feedback;
    LiftControl_State_t direction;
    uint32_t now = HAL_GetTick();
    int32_t target, error;
    bool top_contact = false;
    bool yaw_ready, motor_ready;
    float speed_limit;

    if (remote == NULL || !remote->online)
    {
        lift_wait_reason = LIFT_WAIT_REMOTE;
        calibration_active = false;
        if (!lift_calibrated && !calibration_fault_latched)
        { calibration_armed = true; }
        right_switch_seen = false;
        requested_direction = LIFT_STOPPED;
        lift_hold_target_valid = false;
        LiftControl_ResetStallCheck();
        if (!calibration_fault_latched) { lift_control_state = LIFT_STOPPED; }
        LiftControl_StopAndRelease(now);
        return;
    }
    yaw_ready = CloudTerrace_LiftYawAligned();
    motor_ready = Motor2006_OnlineCheck() &&
                  Motor2006_GetFeedback(&feedback);
    if (!yaw_ready || !motor_ready)
    {
        lift_wait_reason = calibration_fault_latched ? LIFT_WAIT_FAULT :
            !yaw_ready ? LIFT_WAIT_YAW : LIFT_WAIT_MOTOR;
        if (calibration_active)
        {
            calibration_active = false;
            /* 临时掉帧只中断并停机，条件恢复后重新开始找顶部。 */
            if (!calibration_fault_latched) { calibration_armed = true; }
        }
        requested_direction = LIFT_STOPPED;
        right_switch_seen = false;
        if (lift_calibrated) { lift_hold_target_valid = false; }
        LiftControl_ResetStallCheck();
        if (lift_control_state != LIFT_STALLED &&
            lift_control_state != LIFT_AT_LIMIT)
        { lift_control_state = LIFT_STOPPED; }
        LiftControl_StopAndRelease(now);
        return;
    }

    if (!lift_calibrated)
    {
        if (!calibration_active && calibration_armed)
        {
            calibration_active = true;
            calibration_backoff = false;
            calibration_armed = false;
            calibration_drive_started = false;
            calibration_start_ms = now;
            LiftControl_ResetStallCheck();
        }
        if (calibration_active)
        { LiftControl_Calibrate(&feedback, now); }
        else
        {
            lift_wait_reason = LIFT_WAIT_FAULT;
            LiftControl_StopAndRelease(now);
        }
        return;
    }

    lift_wait_reason = LIFT_WAIT_NONE;

    /* 断联或未对准后重新上线时，以当前实测位置作为安全保持点。 */
    if (!lift_hold_target_valid)
    {
        lift_hold_target_encoder_total = feedback.encoder_total;
        lift_hold_target_valid = true;
    }

    if (!remote->lift_mode)
    {
        right_switch_seen = false;
        if (requested_direction != LIFT_STOPPED)
        { lift_hold_target_encoder_total = feedback.encoder_total; }
        requested_direction = LIFT_STOPPED;
    }
    else if (!right_switch_seen)
    {
        /* 首次进入升降模式只记录档位，换档后才选新目标。 */
        previous_right_switch = remote->lift_right_switch;
        right_switch_seen = true;
    }
    else if (remote->lift_right_switch != previous_right_switch)
    {
        previous_right_switch = remote->lift_right_switch;
        requested_direction = remote->lift_right_switch == COMM_RC_SW_DOWN ?
            LIFT_DESCENDING : remote->lift_right_switch == COMM_RC_SW_MID ?
            LIFT_ASCENDING : LIFT_STOPPED;
        lift_hold_target_encoder_total = requested_direction == LIFT_DESCENDING ?
            lift_bottom_encoder_total : requested_direction == LIFT_ASCENDING ?
            lift_top_encoder_total : feedback.encoder_total;
        LiftControl_ResetStallCheck();
    }

    target = lift_hold_target_encoder_total;
    error = target - feedback.encoder_total;
    if ((lift_control_state == LIFT_STALLED ||
         lift_control_state == LIFT_AT_LIMIT) &&
        requested_direction == LIFT_STOPPED)
    {
        LiftControl_StopAndRelease(now);
        return;
    }
    if (LiftControl_PositionArrived(&feedback, target))
    {
        LiftControl_ResetStallCheck();
        lift_control_state = LIFT_READY;
        requested_direction = LIFT_STOPPED;
        if (LiftControl_Abs((int32_t)feedback.speed_rpm) >
                LIFT_CHASSIS_RELEASE_RPM &&
            !LiftControl_ChassisReady(now))
        {
            LiftControl_ResetStallCheck();
            LiftControl_Stop();
            return;
        }
        LiftControl_RunSpeed(0.0f); /* 到位后维持零速，偏离容差会转入位置回位。 */
        LiftControl_SendChassisHold(
            LiftControl_Abs((int32_t)feedback.speed_rpm) >
                LIFT_CHASSIS_RELEASE_RPM, now);
        return;
    }
    direction = LiftControl_DirectionTo(error);
    if (LiftControl_PowerOnLimit(&feedback, direction))
    {
        requested_direction = LIFT_STOPPED;
        lift_control_state = LIFT_AT_LIMIT;
        LiftControl_StopAndRelease(now);
        return;
    }
    if (!LiftControl_ChassisReady(now))
    {
        LiftControl_ResetStallCheck();
        LiftControl_Stop();
        return;
    }
    if (LiftControl_StallCheck(&feedback, direction, now, &top_contact))
    {
        requested_direction = LIFT_STOPPED;
        lift_control_state = LIFT_STALLED;
        LiftControl_StopAndRelease(now);
        return;
    }
    speed_limit = requested_direction == LIFT_STOPPED ?
        LIFT_HOLD_SPEED_RAD_S : direction == LIFT_ASCENDING ?
        LIFT_UP_SPEED_RAD_S : LIFT_DOWN_SPEED_RAD_S;
    lift_control_state = direction;
    LiftControl_PositionDrive(&feedback, target, speed_limit);
}
