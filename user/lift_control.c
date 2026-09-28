#include "lift_control.h"
#include "cloud_terrace.h"
#include "communication.h"
#include "motor2006.h"
#include "application_config.h"
#include "stm32f4xx_hal.h"

#define LIFT_ENCODER_COUNTS_PER_TURN 8192.0f

volatile LiftControl_State_t lift_control_state;
volatile LiftControl_WaitReason_t lift_wait_reason;
volatile bool lift_calibrated;
volatile int32_t lift_top_encoder_total;
volatile int32_t lift_bottom_encoder_total;
volatile bool lift_pitch_nonnegative_required;

static LiftControl_StallSnapshot_t stall_snapshot;
static bool right_switch_seen;
static uint8_t previous_right_switch;
static uint32_t keyboard_lift_seen;
static bool keyboard_lift_pending;
static LiftControl_State_t requested_direction;
static bool calibration_active;
static bool calibration_contact_pending;
static bool calibration_armed;
static bool calibration_fault_latched;
static bool calibration_drive_started;
static bool lift_hold_target_valid;
static int32_t lift_hold_target_encoder_total;
static uint32_t calibration_start_ms;
static bool yaw_stable_timing;
static uint32_t yaw_stable_start_ms;
static bool safety_pause_active;
static uint32_t safety_pause_start_ms;
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
static bool low_mode_blocked;
static bool descent_mode_blocked;
static bool descent_release_timing;
static uint32_t descent_release_start_ms;

static int32_t LiftControl_Abs(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t LiftControl_DownSign(void)
{
    return lift_config.down_direction > 0.0f ? 1 : -1;
}

static void LiftControl_ResetStallCheck(void)
{
    stall_timing = false;
    progress_timing = false;
}

static bool LiftControl_YawStable(uint32_t now)
{
    if (!CloudTerrace_LiftYawAligned())
    {
        yaw_stable_timing = false;
        return false;
    }
    if (!yaw_stable_timing)
    {
        yaw_stable_timing = true;
        yaw_stable_start_ms = now;
    }
    return (uint32_t)(now - yaw_stable_start_ms) >
           lift_config.yaw_stable_ms;
}

static void LiftControl_Stop(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t retry_ms = (lift_control_state == LIFT_STALLED ||
                         lift_control_state == LIFT_AT_LIMIT) ?
                        lift_config.fault_stop_retry_ms : lift_config.stop_retry_ms;

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
         (chassis_hold_request &&
          (uint32_t)(now - chassis_hold_tx_ms) >= lift_config.lock_tx_period_ms)) &&
        Communication_CAN_SendLiftLock(chassis_hold_request,
                                       chassis_hold_sequence) == HAL_OK)
    {
        chassis_hold_tx_seen = true;
        chassis_hold_tx_ms = now;
    }
}

static bool LiftControl_CalibrationChassisReady(uint32_t now)
{
    LiftControl_SendChassisHold(true, now);
    if (!chassis_hold_tx_seen ||
        (uint32_t)(now - chassis_hold_start_ms) <
            lift_config.chassis_lock_settle_ms)
    {
        lift_wait_reason = LIFT_WAIT_CHASSIS_LOCK;
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
                 lift_config.chassis_release_rpm;
    }
    else
    {
        if (!offline_release_timing)
        {
            offline_release_timing = true;
            offline_release_start_ms = now;
        }
        moving = (uint32_t)(now - offline_release_start_ms) <
                 lift_config.offline_release_ms;
    }
    if (chassis_hold_request && (!motor_stopped || moving))
    { LiftControl_SendChassisHold(true, now); }
    else
    { LiftControl_SendChassisHold(false, now); }
}

static void LiftControl_SafetyPause(uint32_t now,
                                    LiftControl_WaitReason_t reason,
                                    bool hold_chassis)
{
    if (!safety_pause_active)
    {
        safety_pause_active = true;
        safety_pause_start_ms = now;
    }
    lift_wait_reason = reason;
    LiftControl_ResetStallCheck();
    if (lift_control_state != LIFT_STALLED &&
        lift_control_state != LIFT_AT_LIMIT)
    { lift_control_state = LIFT_STOPPED; }
    if (hold_chassis)
    {
        LiftControl_Stop();
        LiftControl_SendChassisHold(true, now);
    }
    else
    { LiftControl_StopAndRelease(now); }
}

static void LiftControl_SafetyResume(uint32_t now)
{
    if (!safety_pause_active) { return; }
    if (calibration_active && calibration_drive_started)
    { calibration_start_ms += (uint32_t)(now - safety_pause_start_ms); }
    safety_pause_active = false;
}

static bool LiftControl_PitchRequired(const Motor2006_Feedback_t *feedback)
{
    if (!lift_calibrated) { return false; }
    if (!lift_hold_target_valid) { return false; }
    return LiftControl_Abs(lift_hold_target_encoder_total -
                           lift_bottom_encoder_total) <
               LiftControl_Abs(lift_hold_target_encoder_total -
                           lift_top_encoder_total) ||
           LiftControl_Abs(feedback->encoder_total -
                           lift_bottom_encoder_total) <
               LiftControl_Abs(feedback->encoder_total -
                           lift_top_encoder_total);
}

static bool LiftControl_PowerOnLimit(const Motor2006_Feedback_t *feedback,
                                      LiftControl_State_t direction)
{
    int32_t distance = feedback->encoder_total * LiftControl_DownSign();
    int32_t limit = (int32_t)(lift_config.max_rotor_turns *
                              LIFT_ENCODER_COUNTS_PER_TURN);

    if (direction == LIFT_ASCENDING) { distance = -distance; }
    return limit - distance <= lift_config.limit_stop_margin_counts;
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

// 高电流低速或持续无位移均为堵转；只有持续高电流才可认作顶部接触。
static bool LiftControl_StallCheck(const Motor2006_Feedback_t *feedback,
                                   LiftControl_State_t direction,
                                   uint32_t now, bool *top_contact)
{
    int32_t current_threshold = direction == LIFT_ASCENDING ?
        lift_config.up_stall_current_raw : lift_config.down_stall_current_raw;
    int32_t speed_threshold = direction == LIFT_ASCENDING ?
        lift_config.up_stall_speed_rpm : lift_config.down_stall_speed_rpm;
    uint32_t duration = direction == LIFT_ASCENDING ?
        lift_config.up_stall_time_ms : lift_config.down_stall_time_ms;
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
            lift_config.stall_progress_counts;
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
           lift_config.position_tolerance_counts;
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
                  lift_config.position_kp_rad_s_per_turn;

    if (speed > speed_limit) { speed = speed_limit; }
    else if (speed < -speed_limit) { speed = -speed_limit; }
    if (speed > 0.0f && speed < lift_config.position_min_speed_rad_s)
    { speed = lift_config.position_min_speed_rad_s; }
    else if (speed < 0.0f && speed > -lift_config.position_min_speed_rad_s)
    { speed = -lift_config.position_min_speed_rad_s; }
    LiftControl_RunSpeed(speed);
}

static void LiftControl_CalibrationFail(LiftControl_State_t reason)
{
    calibration_active = false;
    calibration_armed = false; // 本次上线不自动重试。
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

    if ((calibration_drive_started &&
         (uint32_t)(now - calibration_start_ms) >=
             lift_config.calibrate_timeout_ms) ||
        LiftControl_PowerOnLimit(feedback, LIFT_ASCENDING))
    {
        LiftControl_CalibrationFail(LIFT_AT_LIMIT);
        return;
    }
    if (!LiftControl_CalibrationChassisReady(now))
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
        // 碰顶即完成校准；五圈只用于后续高位目标，不在此处回退。
        top = feedback->encoder_total + LiftControl_DownSign() *
            (int32_t)(lift_config.top_clearance_turns *
                      LIFT_ENCODER_COUNTS_PER_TURN);
        bottom = feedback->encoder_total + LiftControl_DownSign() *
            (int32_t)(lift_config.travel_turns *
                      LIFT_ENCODER_COUNTS_PER_TURN);
        if (LiftControl_Abs(top) >= (int32_t)(lift_config.max_rotor_turns *
                LIFT_ENCODER_COUNTS_PER_TURN) ||
            LiftControl_Abs(bottom) >= (int32_t)(lift_config.max_rotor_turns *
                LIFT_ENCODER_COUNTS_PER_TURN))
        {
            LiftControl_CalibrationFail(LIFT_AT_LIMIT);
            return;
        }
        lift_top_encoder_total = top;
        lift_bottom_encoder_total = bottom;
        lift_hold_target_encoder_total = feedback->encoder_total;
        lift_hold_target_valid = false;
        calibration_contact_pending = true;
        calibration_active = false;
        lift_calibrated = true;
        right_switch_seen = false;
        requested_direction = LIFT_STOPPED;
        lift_control_state = LIFT_READY;
        LiftControl_StopAndRelease(now);
        return;
    }
    lift_control_state = LIFT_CALIBRATING_UP;
    LiftControl_RunSpeed(-lift_config.down_direction *
                          lift_config.calibrate_up_speed_rad_s);
}

void LiftControl_Init(void)
{
    lift_control_state = LIFT_STOPPED;
    lift_wait_reason = LIFT_WAIT_REMOTE;
    lift_calibrated = false;
    lift_top_encoder_total = 0;
    lift_bottom_encoder_total = 0;
    lift_pitch_nonnegative_required = false;
    stall_snapshot.valid = false;
    right_switch_seen = false;
    keyboard_lift_seen = 0U;
    keyboard_lift_pending = false;
    previous_right_switch = 0U;
    requested_direction = LIFT_STOPPED;
    calibration_active = false;
    calibration_contact_pending = false;
    calibration_armed = true;
    calibration_fault_latched = false;
    calibration_drive_started = false;
    lift_hold_target_valid = false;
    lift_hold_target_encoder_total = 0;
    calibration_start_ms = 0U;
    yaw_stable_timing = false;
    yaw_stable_start_ms = 0U;
    safety_pause_active = false;
    safety_pause_start_ms = 0U;
    LiftControl_ResetStallCheck();
    motor_stopped = false;
    last_stop_ms = 0U;
    chassis_hold_request = false;
    chassis_hold_sequence = 0U;
    chassis_hold_tx_seen = false; // 上板重启时发送一次释放帧，清除下板可能残留的锁车状态。
    chassis_hold_tx_ms = 0U;
    chassis_hold_start_ms = 0U;
    offline_release_timing = false;
    offline_release_start_ms = 0U;
    low_mode_blocked = false;
    descent_mode_blocked = false;
    descent_release_timing = false;
    descent_release_start_ms = 0U;
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

bool LiftControl_SpecialModesBlocked(const RemoteState_t *remote)
{
    Motor2006_Feedback_t feedback;
    bool feedback_online;
    bool descending_command;
    int32_t down_speed_rpm = 0;
    uint32_t now = HAL_GetTick();

    feedback_online = Motor2006_OnlineCheck() &&
                      Motor2006_GetFeedback(&feedback);
    if (lift_calibrated && feedback_online)
    {
        float bottom_distance_turns =
            (float)LiftControl_Abs(feedback.encoder_total -
                                   lift_bottom_encoder_total) /
            LIFT_ENCODER_COUNTS_PER_TURN;
        float release_turns = lift_config.low_mode_release_turns;
        if (release_turns <= lift_config.low_mode_block_turns)
        { release_turns = lift_config.low_mode_block_turns + 1.0f; }
        if (bottom_distance_turns <= lift_config.low_mode_block_turns)
        { low_mode_blocked = true; }
        else if (bottom_distance_turns >= release_turns)
        { low_mode_blocked = false; }
        down_speed_rpm = (int32_t)feedback.speed_rpm * LiftControl_DownSign();
    }
    else if (!lift_calibrated)
    { low_mode_blocked = false; }

    // 位控保持时的短暂下行纠偏不是一次下降指令，不能据此反复切断自旋。
    descending_command = requested_direction == LIFT_DESCENDING;
    if (remote != NULL && remote->safety.online &&
        remote->safety.lift_enabled)
    {
        if (remote->input.keyboard_active)
        {
            bool pending = keyboard_lift_pending ||
                (((remote->event.lift_toggle_request_count -
                   keyboard_lift_seen) & 1U) != 0U);
            descending_command = descending_command ||
                (pending && lift_hold_target_valid &&
                LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_top_encoder_total) <=
                LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_bottom_encoder_total));
        }
        else if (right_switch_seen &&
            remote->input.lift_right_switch != previous_right_switch &&
            remote->input.lift_right_switch == COMM_RC_SW_DOWN)
        { descending_command = true; }
    }
    // 下行指令立刻生效；中途取消指令后，实测停止下降并稳定 100 ms 才放行。
    if (descending_command ||
        (feedback_online && down_speed_rpm >
            lift_config.chassis_release_rpm * 3))
    {
        descent_mode_blocked = true;
        descent_release_timing = false;
    }
    else if (descent_mode_blocked && feedback_online &&
             down_speed_rpm <= lift_config.chassis_release_rpm)
    {
        if (!descent_release_timing)
        {
            descent_release_timing = true;
            descent_release_start_ms = now;
        }
        else if ((uint32_t)(now - descent_release_start_ms) >= 100U)
        { descent_mode_blocked = false; }
    }
    else
    { descent_release_timing = false; }

    return descent_mode_blocked || low_mode_blocked;
}

bool LiftControl_TurnaroundBlocked(const RemoteState_t *remote)
{
    return calibration_active ||
           lift_control_state == LIFT_CALIBRATING_UP ||
           LiftControl_SpecialModesBlocked(remote);
}

void LiftControl_Update(const RemoteState_t *remote)
{
    Motor2006_Feedback_t feedback;
    LiftControl_State_t direction;
    uint32_t now = HAL_GetTick();
    int32_t target, error;
    bool top_contact = false;
    bool motor_ready;
    bool target_selected = false;
    float speed_limit;

    if (remote != NULL)
    {
        if (remote->input.keyboard_active && remote->safety.lift_enabled &&
            ((remote->event.lift_toggle_request_count - keyboard_lift_seen) & 1U))
        { keyboard_lift_pending = !keyboard_lift_pending; }
        if (!remote->input.keyboard_active || !remote->safety.lift_enabled)
        { keyboard_lift_pending = false; }
        keyboard_lift_seen = remote->event.lift_toggle_request_count;
    }

    if (remote == NULL || !remote->safety.online)
    {
        lift_wait_reason = LIFT_WAIT_REMOTE;
        yaw_stable_timing = false;
        safety_pause_active = false;
        keyboard_lift_pending = false;
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
    motor_ready = Motor2006_OnlineCheck() &&
                  Motor2006_GetFeedback(&feedback);
    if (!motor_ready)
    {
        lift_wait_reason = calibration_fault_latched ? LIFT_WAIT_FAULT :
            LIFT_WAIT_MOTOR;
        yaw_stable_timing = false;
        safety_pause_active = false;
        if (calibration_active)
        {
        calibration_active = false;
            // 电机反馈丢失后重新开始找顶部。
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

    if (!LiftControl_YawStable(now))
    {
        LiftControl_SafetyPause(now, LIFT_WAIT_YAW, calibration_active);
        return;
    }

    if (!lift_calibrated)
    {
        if (!calibration_active && calibration_armed)
        {
            calibration_active = true;
            calibration_armed = false;
            calibration_drive_started = false;
            calibration_start_ms = now;
            LiftControl_ResetStallCheck();
        }
        lift_pitch_nonnegative_required =
            LiftControl_PitchRequired(&feedback);
        if (lift_pitch_nonnegative_required &&
            !CloudTerrace_LiftPitchNonnegative())
        {
            LiftControl_SafetyPause(now, LIFT_WAIT_PITCH, true);
            return;
        }
        LiftControl_SafetyResume(now);
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
    // 校准结束后解除 C2 锁车；正常升降与保持不再要求底盘停车。
    LiftControl_SendChassisHold(false, now);

    // 断联或未对准后重新上线时，以当前实测位置作为安全保持点。
    if (!lift_hold_target_valid && !calibration_contact_pending)
    {
        lift_hold_target_encoder_total = feedback.encoder_total;
        lift_hold_target_valid = true;
    }

    if (!remote->safety.lift_enabled)
    {
        right_switch_seen = false;
        if (requested_direction != LIFT_STOPPED)
        { lift_hold_target_encoder_total = feedback.encoder_total; }
        requested_direction = LIFT_STOPPED;
    }
    else if (remote->input.keyboard_active)
    {
        right_switch_seen = false;
        if (keyboard_lift_pending)
        {
            keyboard_lift_pending = false;
            // B 键按一次，在已校准的高、低目标之间切换。
            if (LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_bottom_encoder_total) <
                LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_top_encoder_total))
            {
                lift_hold_target_encoder_total = lift_top_encoder_total;
                requested_direction = LIFT_ASCENDING;
            }
            else
            {
                lift_hold_target_encoder_total = lift_bottom_encoder_total;
                requested_direction = LIFT_DESCENDING;
            }
            LiftControl_ResetStallCheck();
            target_selected = true;
        }
    }
    else if (!right_switch_seen)
    {
        // 首次进入升降模式只记录档位，换档后才选新目标。
        previous_right_switch = remote->input.lift_right_switch;
        right_switch_seen = true;
    }
    else if (remote->input.lift_right_switch != previous_right_switch)
    {
        previous_right_switch = remote->input.lift_right_switch;
        requested_direction = remote->input.lift_right_switch == COMM_RC_SW_DOWN ?
            LIFT_DESCENDING : remote->input.lift_right_switch == COMM_RC_SW_MID ?
            LIFT_ASCENDING : LIFT_STOPPED;
        lift_hold_target_encoder_total = requested_direction == LIFT_DESCENDING ?
            lift_bottom_encoder_total : requested_direction == LIFT_ASCENDING ?
            lift_top_encoder_total : feedback.encoder_total;
        LiftControl_ResetStallCheck();
        target_selected = requested_direction != LIFT_STOPPED;
    }

    if (calibration_contact_pending)
    {
        if (!target_selected)
        {
            // 等待首次高/低位指令，避免校准后自动顶住机械限位。
            LiftControl_StopAndRelease(now);
            return;
        }
        calibration_contact_pending = false;
        lift_hold_target_valid = true;
    }

    target = lift_hold_target_encoder_total;
    error = target - feedback.encoder_total;
    lift_pitch_nonnegative_required = LiftControl_PitchRequired(&feedback);
    if (lift_pitch_nonnegative_required &&
        !CloudTerrace_LiftPitchNonnegative())
    {
        LiftControl_SafetyPause(now, LIFT_WAIT_PITCH, false);
        return;
    }
    LiftControl_SafetyResume(now);
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
        LiftControl_RunSpeed(0.0f); // 到位后维持零速，偏离容差会转入位置回位。
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
    if (LiftControl_StallCheck(&feedback, direction, now, &top_contact))
    {
        requested_direction = LIFT_STOPPED;
        lift_control_state = LIFT_STALLED;
        LiftControl_StopAndRelease(now);
        return;
    }
    speed_limit = requested_direction == LIFT_STOPPED ?
        lift_config.hold_speed_rad_s : direction == LIFT_ASCENDING ?
        lift_config.up_speed_rad_s : lift_config.down_speed_rad_s;
    lift_control_state = direction;
    LiftControl_PositionDrive(&feedback, target, speed_limit);
}
