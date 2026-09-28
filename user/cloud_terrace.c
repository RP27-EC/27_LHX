#include "cloud_terrace.h"
#include "communication.h"
#include "imu.h"
#include "lift_control.h"
#include "motor4310.h"
#include "PID.h"
#include "peripheral_config.h"
#include "application_config.h"
#include "remote_state.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float target_counts; // Pitch 累计位置目标；摇杆改变目标，松杆保持不变。
} PitchControl_t;

volatile CloudTerrace_HomeState_t cloud_terrace_home_state =
    CLOUD_TERRACE_HOME_WAIT; // 当前归中状态。
static int32_t home_target[MOTOR4310_COUNT]; // 两轴归中时的机械位置目标。
static uint16_t home_stable_cycles; // 两轴连续处于归中误差内的周期数。
static bool targets_initialized; // 常规控制目标是否已从反馈值初始化。
static bool yaw_mechanical_mode; // 是否处于 Yaw 始终朝底盘正前方模式。
static bool yaw_mechanical_in_deadzone; // 机械模式 Yaw 当前是否位于目标角死区。
static int32_t yaw_mechanical_target; // 机械模式选定的物理前/后归中目标。
static int32_t yaw_turn_target; // 本次调头的 Yaw 累计编码器目标。
static uint16_t yaw_turn_stable_cycles; // 调头到位的连续控制周期数。
static bool turn_wheel_armed; // 拨轮回到中位后重新允许下降沿。
static bool turn_requested; // 拨轮事件已触发，等待归中完成后执行。
static PitchControl_t pitch_control; // Pitch 位控目标状态。
static PID_Controller_t yaw_angle_pid; // Yaw 角度外环 PID。
static PID_Controller_t yaw_rate_pid; // Yaw 角速度内环 PID。

volatile bool cloud_yaw_imu_online; // 上板 IMU 是否已标定且在线。
volatile float cloud_yaw_target_deg; // Yaw 位控累计目标角，单位度。
volatile float cloud_yaw_angle_deg; // 当前 IMU 累计 Yaw 角，单位度。
volatile float cloud_yaw_rate_deg_s; // 当前 IMU Yaw 角速度，单位度每秒。
volatile float cloud_yaw_rate_target_deg_s; // 角度外环给出的角速度目标。
volatile int16_t cloud_yaw_torque_raw; // 角速度内环给出的 4310 原始转矩。
volatile bool cloud_turnaround_active; // Yaw 正在转向另一个车头方向。
volatile bool cloud_front_reversed; // 逻辑正方向是否为物理车尾。

static void cloud_reset_home(void)
{
    if (cloud_turnaround_active) { turn_requested = true; }
    cloud_turnaround_active = false;
    yaw_turn_stable_cycles = 0U;
    cloud_terrace_home_state = CLOUD_TERRACE_HOME_WAIT;
    home_stable_cycles = 0U;
    targets_initialized = false;
    yaw_mechanical_mode = false;
    yaw_mechanical_in_deadzone = false;
    yaw_mechanical_target = 0;
    pitch_control.target_counts = 0.0f;
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_rate_pid);
    cloud_yaw_imu_online = false;
    cloud_yaw_target_deg = 0.0f;
    cloud_yaw_angle_deg = 0.0f;
    cloud_yaw_rate_deg_s = 0.0f;
    cloud_yaw_rate_target_deg_s = 0.0f;
    cloud_yaw_torque_raw = 0;
    home_target[MOTOR4310_PITCH] = 0;
    home_target[MOTOR4310_YAW] = 0;
}

void CloudTerrace_Init(void)
{
    PID_Init(&yaw_angle_pid, cloud_config.yaw_angle_kp, cloud_config.yaw_angle_ki,
             cloud_config.yaw_angle_kd, cloud_config.yaw_angle_integral_limit,
             cloud_config.yaw_rate_target_limit_deg_s,
             motor4310_config.control_period_s);
    PID_Init(&yaw_rate_pid, cloud_config.yaw_rate_kp, cloud_config.yaw_rate_ki,
             cloud_config.yaw_rate_kd, cloud_config.yaw_rate_integral_limit,
             cloud_config.yaw_torque_limit_raw, motor4310_config.control_period_s);
    pitch_control.target_counts = 0.0f;
    cloud_reset_home();
}

// 将 [-PI, PI] 电机角换成累计编码器计数，并选择最近的一圈。
static int32_t cloud_nearest_home(float motor_rad, int32_t current)
{
    const int32_t period = (int32_t)(MOTOR4310_ECD_PER_ROUND + 0.5f);
    int32_t target = (int32_t)((motor_rad + MOTOR4310_PMAX) *
                               MOTOR4310_ECD_PER_ROUND /
                               (2.0f * MOTOR4310_PMAX) + 0.5f);
    while (target - current > period / 2) { target -= period; }
    while (target - current < -(period / 2)) { target += period; }
    return target;
}

// 以开机归中的机械零点为基准，取 0°/180° 最近的等效累计目标。
static int32_t cloud_nearest_front_target(bool reversed, int32_t current)
{
    const int32_t period = (int32_t)(MOTOR4310_ECD_PER_ROUND + 0.5f);
    int32_t target = home_target[MOTOR4310_YAW];
    if (reversed) { target += period / 2; }
    while (target - current > period / 2) { target -= period; }
    while (target - current < -(period / 2)) { target += period; }
    return target;
}

// 以归中点为零角度，重力前馈仅保留 k*cos(theta)。
static int16_t cloud_pitch_gravity(const Motor4310_Data_t *feedback)
{
    float theta = (float)(feedback->total_angle - home_target[MOTOR4310_PITCH]) *
                  (2.0f * MOTOR4310_PMAX / MOTOR4310_ECD_PER_ROUND);
    float torque_raw = cloud_config.pitch_gravity_k * cosf(theta) *
                       (4095.0f / (2.0f * CLOUD_MOTOR_TORQUE_MAX_NM));
    if (torque_raw > 2047.0f) { torque_raw = 2047.0f; }
    else if (torque_raw < -2048.0f) { torque_raw = -2048.0f; }
    return (int16_t)torque_raw;
}

static bool cloud_at_home(const Motor4310_Data_t *feedback, int32_t target,
                          int32_t tolerance)
{
    int32_t error = target - feedback->total_angle;
    return error >= -tolerance && error <= tolerance &&
           feedback->speed >= -cloud_config.home_speed_raw_max &&
           feedback->speed <= cloud_config.home_speed_raw_max;
}

static bool cloud_home_step(void)
{
    Motor4310_Data_t pitch, yaw;
    HAL_StatusTypeDef pitch_status, yaw_status;
    int32_t tolerance = Motor4310_PositionToEcd(0.0f,
                                                cloud_config.home_tolerance_deg);

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE) { return true; }
    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &pitch) ||
        !Motor4310_GetFeedback(MOTOR4310_YAW, &yaw)) { return false; }

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_WAIT)
    {
        home_target[MOTOR4310_PITCH] =
            cloud_nearest_home(cloud_config.pitch_home_rad, pitch.total_angle);
        home_target[MOTOR4310_YAW] =
            cloud_nearest_home(cloud_config.yaw_home_rad, yaw.total_angle);
        Motor4310_ResetControl(MOTOR4310_PITCH);
        Motor4310_ResetControl(MOTOR4310_YAW);
        home_stable_cycles = 0U;
        cloud_terrace_home_state = CLOUD_TERRACE_HOME_MOVING;
    }

    pitch_status = Motor4310_PositionControlWithFeedforward(
        MOTOR4310_PITCH, home_target[MOTOR4310_PITCH],
        cloud_pitch_gravity(&pitch));
    yaw_status = Motor4310_PositionControlWithProfile(
        MOTOR4310_YAW, home_target[MOTOR4310_YAW], 0, NULL, false);
    if (pitch_status != HAL_OK || yaw_status != HAL_OK)
    {
        home_stable_cycles = 0U;
        return false;
    }
    if (cloud_at_home(&pitch, home_target[MOTOR4310_PITCH], tolerance) &&
        cloud_at_home(&yaw, home_target[MOTOR4310_YAW], tolerance))
    {
        if (++home_stable_cycles >= cloud_config.home_stable_cycles)
        { cloud_terrace_home_state = CLOUD_TERRACE_HOME_DONE; }
    }
    else { home_stable_cycles = 0U; }
    return cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE;
}

static float cloud_wrap_yaw_deg(float angle_deg)
{
    while (angle_deg > 180.0f) { angle_deg -= 360.0f; }
    while (angle_deg < -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}

static bool cloud_yaw_relative_deg(float *angle_deg)
{
    Motor4310_Data_t yaw;

    if (angle_deg == NULL || !Motor4310_GetFeedback(MOTOR4310_YAW, &yaw))
    { return false; }
    *angle_deg = cloud_wrap_yaw_deg(
        (float)(yaw.total_angle - home_target[MOTOR4310_YAW]) *
        360.0f / MOTOR4310_ECD_PER_ROUND);
    return true;
}

bool CloudTerrace_LiftYawAligned(void)
{
    float relative_deg;

    if (cloud_terrace_home_state != CLOUD_TERRACE_HOME_DONE ||
        cloud_turnaround_active || !Motor4310_OnlineCheck(MOTOR4310_YAW) ||
        !cloud_yaw_relative_deg(&relative_deg))
    { return false; }
    // 升降只认开机归中的物理 0°，调头后的 180° 不算对准。
    return fabsf(relative_deg) < lift_config.yaw_deadzone_deg;
}

bool CloudTerrace_LiftPitchNonnegative(void)
{
    Motor4310_Data_t pitch;

    return cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE &&
           Motor4310_OnlineCheck(MOTOR4310_PITCH) &&
           Motor4310_GetFeedback(MOTOR4310_PITCH, &pitch) &&
           pitch.total_angle >= home_target[MOTOR4310_PITCH];
}

static void cloud_turn_wheel_update(int16_t wheel, bool allow_turn)
{
    if (wheel > -cloud_config.turn_wheel_rearm_raw)
    { turn_wheel_armed = true; }
    else if (turn_wheel_armed && wheel <= -cloud_config.turn_wheel_trigger_raw)
    {
        turn_wheel_armed = false;
        if (allow_turn && !cloud_turnaround_active)
        { turn_requested = true; }
    }
}

static bool cloud_turn_start(void)
{
    Motor4310_Data_t yaw;
    float relative_deg;

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        !cloud_yaw_relative_deg(&relative_deg)) { return false; }
    // 当前较近的方向若为车头，则本次转向车尾；反之转向车头。
    cloud_front_reversed = fabsf(relative_deg) < cloud_config.front_switch_deg;
    yaw_turn_target = cloud_nearest_front_target(cloud_front_reversed,
                                                 yaw.total_angle);
    yaw_turn_stable_cycles = 0U;
    turn_requested = false;
    cloud_turnaround_active = true;
    yaw_mechanical_mode = false;
    yaw_mechanical_in_deadzone = false;
    cloud_yaw_imu_online = false; // 完成后惯性控制从实际朝向重新接管。
    Motor4310_ResetControl(MOTOR4310_YAW);
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_rate_pid);
    return true;
}

static void cloud_turn_step(void)
{
    Motor4310_Data_t yaw;
    int32_t tolerance = Motor4310_PositionToEcd(0.0f,
                                                cloud_config.turn_tolerance_deg);
    int32_t error;

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        Motor4310_PositionControlWithProfile(
            MOTOR4310_YAW, yaw_turn_target, 0, NULL, true) != HAL_OK)
    {
        yaw_turn_stable_cycles = 0U;
        return;
    }
    error = yaw_turn_target - yaw.total_angle;
    if (error >= -tolerance && error <= tolerance &&
        yaw.speed >= -cloud_config.turn_speed_raw_max &&
        yaw.speed <= cloud_config.turn_speed_raw_max)
    {
        if (++yaw_turn_stable_cycles >= cloud_config.turn_stable_cycles)
        {
            cloud_turnaround_active = false;
            Motor4310_ResetControl(MOTOR4310_YAW);
        }
    }
    else { yaw_turn_stable_cycles = 0U; }
}

static void cloud_control_yaw(int16_t input)
{
    GimbalImu_Data_t imu;
    float torque;

    if (yaw_mechanical_mode)
    {
        // 切回惯性控制时从当前 IMU 朝向重建目标。
        Motor4310_ResetControl(MOTOR4310_YAW);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = false;
        yaw_mechanical_in_deadzone = false;
    }

    if (input >= -cloud_config.rc_speed_enter && input <= cloud_config.rc_speed_enter)
    { input = 0; }
    if (!GimbalImu_Get(&imu))
    {
        cloud_yaw_imu_online = false;
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        cloud_yaw_torque_raw = 0;
        (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW, 0);
        return;
    }

    if (!cloud_yaw_imu_online)
    {
        // IMU 恢复时从当前朝向重新接管，避免追赶旧目标突跳。
        cloud_yaw_target_deg = imu.yaw_total_deg;
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
    }
    cloud_yaw_imu_online = true;
    cloud_yaw_angle_deg = imu.yaw_total_deg;
    cloud_yaw_rate_deg_s = imu.yaw_rate_deg_s;
    // 摇杆改变惯性系角度目标；松杆后目标不变，云台稳向。
    cloud_yaw_target_deg += cloud_config.yaw_rc_direction *
        (float)input / CLOUD_RC_MAX_VALUE * cloud_config.yaw_command_rate_deg_s *
        motor4310_config.control_period_s;
    PID_UpdateParameters(&yaw_angle_pid,
        cloud_config.yaw_angle_kp, cloud_config.yaw_angle_ki,
        cloud_config.yaw_angle_kd, cloud_config.yaw_angle_integral_limit,
        cloud_config.yaw_rate_target_limit_deg_s,
        motor4310_config.control_period_s);
    PID_UpdateParameters(&yaw_rate_pid,
        cloud_config.yaw_rate_kp, cloud_config.yaw_rate_ki,
        cloud_config.yaw_rate_kd, cloud_config.yaw_rate_integral_limit,
        cloud_config.yaw_torque_limit_raw,
        motor4310_config.control_period_s);
    cloud_yaw_rate_target_deg_s = PID_Calc(
        &yaw_angle_pid, cloud_yaw_target_deg, imu.yaw_total_deg);
    torque = PID_Calc(&yaw_rate_pid, cloud_yaw_rate_target_deg_s,
                      imu.yaw_rate_deg_s);
    // 跟随和小陀螺的 IMU 稳向控制不叠加固定前馈。
    cloud_yaw_torque_raw = (int16_t)torque;
    (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW,
                                      cloud_yaw_torque_raw);
}

static void cloud_control_yaw_mechanical(bool use_near_pid)
{
    Motor4310_Data_t yaw;
    float error_deg;

    if (!yaw_mechanical_mode)
    {
        if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw))
        {
            cloud_yaw_torque_raw = 0;
            return;
        }

        // 前/后方向只改变目标，不重新定义开机校准的机械零点。
        cloud_front_reversed = fabsf(cloud_wrap_yaw_deg(
            (float)(yaw.total_angle - home_target[MOTOR4310_YAW]) *
            360.0f / MOTOR4310_ECD_PER_ROUND)) >= cloud_config.front_switch_deg;
        yaw_mechanical_target = cloud_nearest_front_target(
            cloud_front_reversed, yaw.total_angle);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        Motor4310_ResetControl(MOTOR4310_YAW);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = true;
        yaw_mechanical_in_deadzone = false;
    }

    cloud_yaw_rate_target_deg_s = 0.0f;
    cloud_yaw_torque_raw = 0;
    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw)) { return; }
    // 按当前前/后归中目标的误差选档，180° 调头后也可进入近点控制。
    error_deg = fabsf((float)(yaw_mechanical_target - yaw.total_angle) *
                      360.0f / MOTOR4310_ECD_PER_ROUND);
    if (use_near_pid &&
        error_deg <= cloud_config.mechanical_yaw_deadzone_deg)
    {
        if (!yaw_mechanical_in_deadzone)
        {
            Motor4310_ResetControl(MOTOR4310_YAW);
            yaw_mechanical_in_deadzone = true;
        }
        // 位置目标跟随当前反馈，只保留近点速度环抑制惯性，不追逐 ±1° 内的误差。
        Motor4310_PidProfile_t near_pid = motor4310_config.yaw_near_pid;
        (void)Motor4310_PositionControlWithProfile(
            MOTOR4310_YAW, yaw.total_angle, 0, &near_pid, false);
        return;
    }
    if (yaw_mechanical_in_deadzone)
    {
        Motor4310_ResetControl(MOTOR4310_YAW);
        yaw_mechanical_in_deadzone = false;
    }
    if (use_near_pid && error_deg < cloud_config.mechanical_yaw_near_deg)
    {
        Motor4310_PidProfile_t near_pid = motor4310_config.yaw_near_pid;
        (void)Motor4310_PositionControlWithProfile(
            MOTOR4310_YAW, yaw_mechanical_target, 0, &near_pid, true);
    }
    else
    {
        (void)Motor4310_PositionControlWithProfile(
            MOTOR4310_YAW, yaw_mechanical_target, 0, NULL, use_near_pid);
    }
}

static void cloud_send_yaw_angle(bool allow_turn)
{
    float relative_deg;

    if (!cloud_yaw_relative_deg(&relative_deg)) { return; }
    // 传机械归中后的相对车头角，而非电机编码器原始零点。
    (void)Communication_CAN_SendYawState(relative_deg,
                                         cloud_turnaround_active, allow_turn);
}

static void cloud_control_pitch(int16_t input)
{
    Motor4310_Data_t feedback;
    int32_t lower, upper;
    float lead_counts;
    int16_t gravity;

    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &feedback)) { return; }
    lower = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, cloud_config.pitch_min_deg);
    if (lift_pitch_nonnegative_required)
    {
        int32_t lift_lower = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f,
                cloud_config.lift_pitch_clearance_deg > 0.0f ?
                cloud_config.lift_pitch_clearance_deg : 0.0f);
        if (lower < lift_lower) { lower = lift_lower; }
    }
    upper = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, cloud_config.pitch_max_deg);
    gravity = cloud_pitch_gravity(&feedback);
    if (input > cloud_config.rc_speed_enter ||
        input < -cloud_config.rc_speed_enter)
    {
        // 遥控只积分位置目标，不切换到底层速度控制。
        pitch_control.target_counts += (float)input / CLOUD_RC_MAX_VALUE *
            cloud_config.pitch_command_rate_deg_s *
            motor4310_config.control_period_s *
            (MOTOR4310_ECD_PER_ROUND / 360.0f);
        lead_counts = cloud_config.pitch_target_lead_deg *
                      (MOTOR4310_ECD_PER_ROUND / 360.0f);
        if (lead_counts < 0.0f) { lead_counts = 0.0f; }
        if (pitch_control.target_counts > (float)feedback.total_angle + lead_counts)
        { pitch_control.target_counts = (float)feedback.total_angle + lead_counts; }
        else if (pitch_control.target_counts < (float)feedback.total_angle - lead_counts)
        { pitch_control.target_counts = (float)feedback.total_angle - lead_counts; }
    }
    if (pitch_control.target_counts < (float)lower)
    { pitch_control.target_counts = (float)lower; }
    if (pitch_control.target_counts > (float)upper)
    { pitch_control.target_counts = (float)upper; }
    (void)Motor4310_PositionControlWithFeedforward(
        MOTOR4310_PITCH, (int32_t)pitch_control.target_counts, gravity);
}

void CloudTerrace_Update(void)
{
    RemoteState_t remote;
    HAL_StatusTypeDef pitch_enable, yaw_enable;
    bool turn_blocked;
    bool lift_modes_blocked;

    Motor4310_Heartbeat();
    RemoteState_Get(&remote);
    if (remote.mode.chassis != REMOTE_MODE_FOLLOW &&
        remote.mode.chassis != REMOTE_MODE_MECHANICAL &&
        remote.mode.chassis != REMOTE_MODE_SPIN)
    {
        // 遥控断联或档位无效：取消调头并让两轴失能。
        cloud_reset_home();
        turn_requested = false;
        turn_wheel_armed = false;
        cloud_front_reversed = false;
        (void)Motor4310_DisableMotor(MOTOR4310_PITCH);
        (void)Motor4310_DisableMotor(MOTOR4310_YAW);
        return;
    }

    lift_modes_blocked = LiftControl_SpecialModesBlocked(&remote);
    turn_blocked = (remote.mode.chassis == REMOTE_MODE_SPIN &&
                    remote.safety.spin_enabled) ||
                   LiftControl_TurnaroundBlocked(&remote);
    if (lift_modes_blocked && cloud_turnaround_active)
    {
        cloud_turnaround_active = false;
        yaw_turn_stable_cycles = 0U;
        turn_requested = false;
        Motor4310_ResetControl(MOTOR4310_YAW);
    }
    cloud_turn_wheel_update(remote.input.channel[4], !turn_blocked);
    if (turn_blocked && !cloud_turnaround_active)
    { turn_requested = false; }

    pitch_enable = Motor4310_EnableMotor(MOTOR4310_PITCH);
    yaw_enable = Motor4310_EnableMotor(MOTOR4310_YAW);
    if (pitch_enable != HAL_OK || yaw_enable != HAL_OK ||
        !Motor4310_AllOnline())
    {
        // 任意一轴掉线即停止两轴闭环，恢复后重新归中。
        cloud_reset_home();
        if (Motor4310_OnlineCheck(MOTOR4310_PITCH))
        { (void)Motor4310_SetTorqueRawMotor(MOTOR4310_PITCH, 0); }
        if (Motor4310_OnlineCheck(MOTOR4310_YAW))
        { (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW, 0); }
        return;
    }
    if (!cloud_home_step()) { return; }
    if (!targets_initialized)
    {
        pitch_control.target_counts = (float)home_target[MOTOR4310_PITCH];
        targets_initialized = true;
    }

    if (turn_requested && !cloud_turnaround_active && !turn_blocked)
    { (void)cloud_turn_start(); }
    if (cloud_turnaround_active)
    {
        // 下板已由同一拨轮边沿先行停车；此时只让云台轴执行控制。
        cloud_turn_step();
        cloud_control_pitch(remote.input.channel[1]);
        cloud_send_yaw_angle(!turn_blocked);
        return;
    }

    if (!lift_calibrated || remote.mode.chassis == REMOTE_MODE_MECHANICAL)
    {
        // 首次校准前持续对准机械正方向，避免稳向停在升降死区外。
        cloud_control_yaw_mechanical(remote.mode.chassis == REMOTE_MODE_MECHANICAL);
    }
    else
    {
        // 跟随及小陀螺模式使用惯性系 Yaw 控制。
        cloud_control_yaw(-remote.input.channel[0]);
    }
    cloud_control_pitch(remote.input.channel[1]);
    cloud_send_yaw_angle(!turn_blocked);
}
