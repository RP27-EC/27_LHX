#include "lift_control.h"
#include "cloud_terrace.h"
#include "communication.h"
#include "motor2006.h"
#include "motor4310.h"
#include "imu.h"
#include "application_config.h"
#include "stm32f4xx_hal.h"

#define LIFT_ENCODER_COUNTS_PER_TURN 8192.0f // 升降电机转子每圈编码器计数。

volatile LiftControl_State_t lift_control_state; // 升降当前动作状态。
volatile LiftControl_WaitReason_t lift_wait_reason; // 当前等待或停机原因。
volatile bool lift_calibrated; // 顶部基准和行程目标是否已建立。
volatile int32_t lift_top_encoder_total; // 当前高位目标，累计编码器计数。
volatile int32_t lift_top_contact_encoder_total; // 最近确认的机械顶部基准。
volatile int32_t lift_bottom_encoder_total; // 校准后的低位目标，累计编码器计数。
volatile bool lift_pitch_nonnegative_required; // 是否要求 Pitch 抬至机械安全下限。
volatile LiftSafetyState_t lift_safety_state; // 云台、发射和底盘共用的安全快照。

static LiftControl_StallSnapshot_t stall_snapshot; // 最近一次堵转反馈。
static bool right_switch_seen; // 已记录升降档内的首次拨杆位置。
static uint8_t previous_right_switch; // 上次处理的升降拨杆档位。
static uint32_t keyboard_lift_seen; // 已消费的键鼠升降事件序号。
static bool keyboard_lift_pending; // 等待归零后执行的键鼠升降请求。
static LiftControl_State_t requested_direction; // 主动升降方向；保持纠偏不改变此项。
static bool calibration_active; // 正在自动寻找顶部。
static bool calibration_armed; // 允许开始一次顶部校准。
static bool calibration_fault_latched; // 校准失败后禁止自动重试。
static bool calibration_drive_started; // 校准已开始驱动电机，开始计算超时。
static bool lift_hold_target_valid; // 累计位置保持目标是否有效。
static int32_t lift_hold_target_encoder_total; // 本次升降或保持的位置目标。
static uint32_t calibration_start_ms; // 校准驱动起始时间。
static bool yaw_stable_timing; // Yaw 正在累计连续归零时间。
static uint32_t yaw_stable_start_ms; // 本次 Yaw 连续归零的起点。
static bool safety_pause_active; // 因联锁暂时停止升降。
static uint32_t safety_pause_start_ms; // 安全暂停起点，用于扣除校准等待时间。
static bool stall_timing; // 高电流低速判据正在计时。
static uint32_t stall_start_ms; // 本次高电流低速判据的起点。
static bool progress_timing; // 位移进度检查窗口有效。
static uint32_t progress_start_ms; // 当前位移检查窗口起点。
static int32_t progress_start_counts; // 位移检查窗口起始位置。
static bool progress_high_current_all; // 检查窗口内是否持续为高电流。
static bool motor_stopped; // 停机命令是否已成功入队。
static uint32_t last_stop_ms; // 最近一次停机命令入队时间。
static bool chassis_hold_request; // 当前是否请求下板锁车。
static uint8_t chassis_hold_sequence; // 锁车请求变化的事件序号。
static bool chassis_hold_tx_seen; // 当前锁车状态已成功入队。
static uint32_t chassis_hold_tx_ms; // 最近一次锁车状态发送时间。
static uint32_t chassis_hold_start_ms; // 本次锁车等待的起点。
static bool offline_release_timing; // 电机离线后的延时释放正在计时。
static uint32_t offline_release_start_ms; // 本次离线释放计时起点。
static bool upper_mode_zone; // 顶部安全区的迟滞状态。
static bool descent_mode_blocked; // 下降联锁，停止稳定后解除。
static bool down_motion_timing; // 实测持续下行正在计时。
static uint32_t down_motion_start_ms; // 本次实测下行计时起点。
static bool descent_release_timing; // 下降结束后的稳定计时有效。
static uint32_t descent_release_start_ms; // 本次下降联锁释放计时起点。

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

static bool LiftControl_PitchRequired(void)
{
    if (!lift_calibrated) { return false; }
    if (!lift_hold_target_valid) { return false; }
    // 下降开始前先抬 Pitch；离开顶部安全区后保持抬起直到归位。
    return requested_direction == LIFT_DESCENDING || !upper_mode_zone;
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

// 主动升降检查无进度；保持纠偏只检查持续高电流低速，避免低速保持误报。
static bool LiftControl_StallCheck(const Motor2006_Feedback_t *feedback,
                                   LiftControl_State_t direction,
                                   uint32_t now, bool check_progress, bool *top_contact)
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
    no_progress = check_progress &&
        (uint32_t)(now - progress_start_ms) >= duration &&
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
    int32_t tolerance = lift_config.position_tolerance_counts;

    if (lift_calibrated && target == lift_top_encoder_total &&
        lift_config.top_arrival_tolerance_counts > tolerance)
    { tolerance = lift_config.top_arrival_tolerance_counts; }
    return LiftControl_Abs(target - feedback->encoder_total) <=
           tolerance;
}

static LiftControl_State_t LiftControl_DirectionTo(int32_t error)
{
    return error * LiftControl_DownSign() > 0 ?
        LIFT_DESCENDING : LIFT_ASCENDING;
}

static void LiftControl_PositionDrive(const Motor2006_Feedback_t *feedback,
                                       int32_t target, float speed_limit,
                                       bool active_move)
{
    float speed = (float)(target - feedback->encoder_total) /
                  LIFT_ENCODER_COUNTS_PER_TURN *
                  lift_config.position_kp_rad_s_per_turn;

    if (speed > speed_limit) { speed = speed_limit; }
    else if (speed < -speed_limit) { speed = -speed_limit; }
    // 最小速度只用于主动升降，保持时让纠偏速度随误差减小。
    if (active_move && speed > 0.0f && speed < lift_config.position_min_speed_rad_s)
    { speed = lift_config.position_min_speed_rad_s; }
    else if (active_move && speed < 0.0f && speed > -lift_config.position_min_speed_rad_s)
    { speed = -lift_config.position_min_speed_rad_s; }
    if (speed > speed_limit) { speed = speed_limit; }
    else if (speed < -speed_limit) { speed = -speed_limit; }
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
                               true, &top_contact))
    {
        if (!top_contact)
        {
            LiftControl_CalibrationFail(LIFT_STALLED);
            return;
        }
        // 碰顶位置即高位目标；低位仍从该机械顶点向下计算。
        top = feedback->encoder_total;
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
        lift_top_contact_encoder_total = feedback->encoder_total;
        lift_bottom_encoder_total = bottom;
        lift_hold_target_encoder_total = top;
        lift_hold_target_valid = true;
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
    lift_top_contact_encoder_total = 0;
    lift_bottom_encoder_total = 0;
    lift_pitch_nonnegative_required = false;
    stall_snapshot.valid = false;
    right_switch_seen = false;
    keyboard_lift_seen = 0U;
    keyboard_lift_pending = false;
    previous_right_switch = 0U;
    requested_direction = LIFT_STOPPED;
    calibration_active = false;
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
    lift_safety_state.from_top_turns = -1.0f;
    lift_safety_state.position_valid = false;
    lift_safety_state.upper_zone = false;
    lift_safety_state.bottom_mode_blocked = false;
    lift_safety_state.yaw_home_required = false;
    lift_safety_state.descending = false;
    lift_safety_state.special_allowed = false;
    lift_safety_state.spin_allowed = false;
    lift_safety_state.shoot_allowed = false;
    lift_safety_state.update_ms = 0U;
    upper_mode_zone = false;
    descent_mode_blocked = false;
    down_motion_timing = false;
    down_motion_start_ms = 0U;
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

void LiftControl_SafetyUpdate(const RemoteState_t *remote)
{
    Motor2006_Feedback_t feedback;
    GimbalImu_Data_t imu;
    LiftSafetyState_t next = {0};
    bool feedback_online;
    bool descending_command;
    bool pending_lift_command = false;
    bool confirmed_down_motion = false;
    bool pending_descent = false;
    bool holding_position = false; // 顶部区内无主动升降请求，电机可继续保持纠偏。
    bool imu_ready;
    int32_t down_speed_rpm = 0;
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    float exit_turns = lift_config.special_exit_from_top_turns;
    float top_margin_turns = (float)lift_config.top_arrival_tolerance_counts /
                             LIFT_ENCODER_COUNTS_PER_TURN;
    float bottom_margin_turns = (float)lift_config.position_tolerance_counts /
                                LIFT_ENCODER_COUNTS_PER_TURN;

    next.from_top_turns = -1.0f;
    next.update_ms = now;
    feedback_online = Motor2006_OnlineCheck() &&
                      Motor2006_GetFeedback(&feedback);
    if (lift_calibrated && feedback_online)
    {
        next.from_top_turns = (float)(feedback.encoder_total -
            lift_top_contact_encoder_total) * LiftControl_DownSign() /
            LIFT_ENCODER_COUNTS_PER_TURN;
        next.position_valid = next.from_top_turns >= -top_margin_turns &&
            next.from_top_turns <= lift_config.travel_turns +
                                   bottom_margin_turns &&
            !calibration_active &&
            lift_control_state != LIFT_STALLED &&
            lift_control_state != LIFT_AT_LIMIT;
        down_speed_rpm = (int32_t)feedback.speed_rpm * LiftControl_DownSign();
    }
    if (exit_turns <= lift_config.special_enter_from_top_turns)
    { exit_turns = lift_config.special_enter_from_top_turns + 1.0f; }
    if (!next.position_valid || lift_config.special_enter_from_top_turns <= 0.0f)
    { upper_mode_zone = false; }
    else if (next.from_top_turns <= lift_config.special_enter_from_top_turns)
    { upper_mode_zone = true; }
    else if (next.from_top_turns > exit_turns)
    { upper_mode_zone = false; }
    next.upper_zone = upper_mode_zone;
    if (lift_calibrated && feedback_online && lift_hold_target_valid)
    {
        // 到位先撤销运动请求，云台当周期即可接管。
        if (requested_direction != LIFT_STOPPED &&
            LiftControl_PositionArrived(&feedback, lift_hold_target_encoder_total))
        { requested_direction = LIFT_STOPPED; }
        if (remote != NULL &&
            (!remote->safety.online || !remote->safety.lift_enabled ||
             (!remote->input.keyboard_active &&
              remote->input.lift_right_switch == COMM_RC_SW_UP)))
        {
            if (requested_direction != LIFT_STOPPED &&
                !(requested_direction == LIFT_ASCENDING && next.upper_zone))
            { lift_hold_target_encoder_total = feedback.encoder_total; }
            // 离开升降档或回停机档不保留回零联锁；顶部目标仍可低速保持。
            requested_direction = LIFT_STOPPED;
            keyboard_lift_pending = false;
            right_switch_seen = false;
        }
        holding_position = next.upper_zone && requested_direction == LIFT_STOPPED;
    }
    // 低位锁定只由已校准的位置解除；反馈暂失效时保留上次锁定状态。
    next.bottom_mode_blocked = lift_safety_state.bottom_mode_blocked;
    if (lift_calibrated && feedback_online)
    {
        float enter_turns = lift_config.bottom_mode_enter_turns;
        float exit_turns = lift_config.bottom_mode_exit_turns;
        if (enter_turns < 0.0f) { enter_turns = 0.0f; }
        if (exit_turns <= enter_turns) { exit_turns = enter_turns + 1.0f; }
        if (next.from_top_turns >= lift_config.travel_turns - enter_turns)
        { next.bottom_mode_blocked = true; }
        else if (next.from_top_turns <= lift_config.travel_turns - exit_turns)
        { next.bottom_mode_blocked = false; }
    }
    // 目标与实际运动分开：短暂的位控纠偏不算新的下降指令。
    // 顶部自由瞄准期间也持续更新归零计时，避免下一次升降沿用旧的稳定记录。
    (void)LiftControl_YawStable(now);
    descending_command = requested_direction == LIFT_DESCENDING;
    if (remote != NULL && remote->safety.online &&
        remote->safety.lift_enabled)
    {
        if (remote->input.keyboard_active)
        {
            bool pending = keyboard_lift_pending ||
                (((remote->event.lift_toggle_request_count -
                   keyboard_lift_seen) & 1U) != 0U);
            pending_lift_command = pending;
            pending_descent = pending && lift_hold_target_valid &&
                LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_top_encoder_total) <=
                LiftControl_Abs(lift_hold_target_encoder_total -
                                lift_bottom_encoder_total);
        }
        else if (right_switch_seen &&
            remote->input.lift_right_switch != previous_right_switch)
        {
            pending_lift_command =
                remote->input.lift_right_switch == COMM_RC_SW_DOWN ||
                remote->input.lift_right_switch == COMM_RC_SW_MID;
            pending_descent =
                remote->input.lift_right_switch == COMM_RC_SW_DOWN;
        }
    }
    descending_command = descending_command || pending_descent;
    next.yaw_home_required = lift_calibrated &&
        (requested_direction != LIFT_STOPPED || pending_lift_command);
    // 安全快照先于云台控制更新，使新下降指令当周期就能抬起 Pitch。
    lift_pitch_nonnegative_required = remote != NULL &&
        remote->safety.online && feedback_online &&
        (pending_descent || LiftControl_PitchRequired());
    // 遥控下降请求立即撤销许可；高位位控的短暂下行纠偏不算下降。
    if (feedback_online && !holding_position && down_speed_rpm >
        lift_config.special_down_speed_enter_rpm)
    {
        if (!down_motion_timing)
        {
            down_motion_timing = true;
            down_motion_start_ms = now;
        }
        confirmed_down_motion = (uint32_t)(now - down_motion_start_ms) >=
            lift_config.special_down_motion_confirm_ms;
    }
    else { down_motion_timing = false; }
    if (descending_command || confirmed_down_motion)
    {
        descent_mode_blocked = true;
        descent_release_timing = false;
    }
    else if (descent_mode_blocked && feedback_online &&
             (holding_position ||
              down_speed_rpm <= lift_config.special_down_speed_release_rpm))
    {
        if (!descent_release_timing)
        {
            descent_release_timing = true;
            descent_release_start_ms = now;
        }
        else if ((uint32_t)(now - descent_release_start_ms) >=
                     lift_config.special_down_stop_stable_ms)
        { descent_mode_blocked = false; }
    }
    else
    { descent_release_timing = false; }

    next.descending = descent_mode_blocked;
    imu_ready = GimbalImu_Get(&imu);
    next.special_allowed = remote != NULL && remote->safety.online &&
        next.upper_zone && !next.descending && !next.yaw_home_required &&
        cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE &&
        Motor4310_AllOnline() &&
        (remote->mode.chassis == REMOTE_MODE_MECHANICAL || imu_ready);
    next.spin_allowed = next.special_allowed && imu_ready;
    // 键鼠机械模式可用 B 升降、F 发射；仅物理左下档独占右拨杆。
    next.shoot_allowed = next.special_allowed &&
        !(remote->safety.lift_enabled && !remote->input.keyboard_active);

    primask = __get_PRIMASK();
    __disable_irq();
    lift_safety_state = next;
    __set_PRIMASK(primask);
}

bool LiftControl_SafetyGet(LiftSafetyState_t *state)
{
    uint32_t primask;

    if (state == NULL) { return false; }
    primask = __get_PRIMASK();
    __disable_irq();
    *state = lift_safety_state;
    __set_PRIMASK(primask);
    if (state->update_ms == 0U ||
        (uint32_t)(HAL_GetTick() - state->update_ms) >=
            lift_config.special_state_timeout_ms)
    {
        state->position_valid = false;
        state->upper_zone = false;
        state->descending = true;
        state->yaw_home_required = true;
        state->special_allowed = false;
        state->spin_allowed = false;
        state->shoot_allowed = false;
        return false;
    }
    return true;
}

void LiftControl_Update(const RemoteState_t *remote)
{
    Motor2006_Feedback_t feedback;
    LiftSafetyState_t safety;
    LiftControl_State_t direction;
    uint32_t now = HAL_GetTick();
    int32_t target, error;
    bool top_contact = false;
    bool motor_ready;
    bool safety_valid;
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

    safety_valid = LiftControl_SafetyGet(&safety);
    if (!safety_valid)
    {
        // 云台计算耗时不应让同一控制周期的升降指令被误判为旧快照。
        LiftControl_SafetyUpdate(remote);
        safety_valid = LiftControl_SafetyGet(&safety);
    }
    if (!safety_valid)
    {
        // 云台任务超期时不沿用旧目标；恢复后仍须新的换档/按键事件。
        keyboard_lift_pending = false;
        right_switch_seen = false;
        requested_direction = LIFT_STOPPED;
        if (lift_calibrated)
        {
            lift_hold_target_encoder_total = feedback.encoder_total;
            lift_hold_target_valid = true;
        }
        LiftControl_SafetyPause(now, LIFT_WAIT_SAFETY_STATE,
                                calibration_active);
        return;
    }

    // 新升降边沿在 Yaw 未归零时保持待执行；归零稳定后才读取目标并驱动 2006。
    if ((!lift_calibrated || safety.yaw_home_required) &&
        !LiftControl_YawStable(now))
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
            LiftControl_PitchRequired();
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
    if (!lift_hold_target_valid)
    {
        lift_hold_target_encoder_total = feedback.encoder_total;
        lift_hold_target_valid = true;
    }

    if (!remote->safety.lift_enabled)
    {
        right_switch_seen = false;
        if (requested_direction != LIFT_STOPPED &&
            !(requested_direction == LIFT_ASCENDING && safety.upper_zone))
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
        if (remote->input.lift_right_switch == COMM_RC_SW_DOWN)
        {
            requested_direction = LIFT_DESCENDING;
            lift_hold_target_encoder_total = lift_bottom_encoder_total;
        }
        else if (remote->input.lift_right_switch == COMM_RC_SW_MID)
        {
            requested_direction = LIFT_ASCENDING;
            lift_hold_target_encoder_total = lift_top_encoder_total;
        }
        else if (!(requested_direction == LIFT_ASCENDING && safety.upper_zone))
        {
            // 中途停机保持当前位置；已经到位则保留原来的保持目标。
            if (requested_direction != LIFT_STOPPED)
            { lift_hold_target_encoder_total = feedback.encoder_total; }
            requested_direction = LIFT_STOPPED;
        }
        LiftControl_ResetStallCheck();
    }

    target = lift_hold_target_encoder_total;
    error = target - feedback.encoder_total;
    lift_pitch_nonnegative_required = LiftControl_PitchRequired();
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
        lift_wait_reason = LIFT_WAIT_FAULT;
        LiftControl_StopAndRelease(now);
        return;
    }
    if (LiftControl_PositionArrived(&feedback, target))
    {
        LiftControl_ResetStallCheck();
        lift_control_state = LIFT_READY;
        requested_direction = LIFT_STOPPED;
        // 到位容差只结束主动动作，容差内仍持续做位置保持。
        LiftControl_PositionDrive(&feedback, target,
                                  lift_config.hold_speed_rad_s, false);
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
    if (LiftControl_StallCheck(&feedback, direction, now,
                               requested_direction != LIFT_STOPPED, &top_contact))
    {
        if (direction == LIFT_ASCENDING &&
            target == lift_top_encoder_total &&
            (top_contact ||
             (requested_direction == LIFT_ASCENDING &&
              LiftControl_Abs((int32_t)feedback.speed_rpm) <=
                  lift_config.up_stall_speed_rpm)) &&
            lift_config.top_contact_window_turns > 0.0f &&
            LiftControl_Abs(feedback.encoder_total -
                            lift_top_contact_encoder_total) <=
                (int32_t)(lift_config.top_contact_window_turns *
                          LIFT_ENCODER_COUNTS_PER_TURN))
        {
            // 已知顶部附近停住按到位处理；只有高电流接触才修正机械顶点。
            if (top_contact)
            {
                lift_top_encoder_total = feedback.encoder_total;
                lift_top_contact_encoder_total = feedback.encoder_total;
            }
            lift_hold_target_encoder_total = feedback.encoder_total;
            requested_direction = LIFT_STOPPED;
            lift_control_state = LIFT_READY;
            stall_snapshot.valid = false;
            LiftControl_RunSpeed(0.0f);
            return;
        }
        requested_direction = LIFT_STOPPED;
        lift_control_state = LIFT_STALLED;
        lift_wait_reason = LIFT_WAIT_FAULT;
        LiftControl_StopAndRelease(now);
        return;
    }
    speed_limit = requested_direction == LIFT_STOPPED ?
        lift_config.hold_speed_rad_s : direction == LIFT_ASCENDING ?
        lift_config.up_speed_rad_s : lift_config.down_speed_rad_s;
    lift_control_state = direction;
    LiftControl_PositionDrive(&feedback, target, speed_limit,
                              requested_direction != LIFT_STOPPED);
}
