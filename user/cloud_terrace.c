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
static int32_t yaw_mechanical_target; // 机械模式选定的物理前/后归中目标。
static int32_t yaw_turn_target; // 本次调头的 Yaw 累计编码器目标。
static int32_t yaw_turn_start_counts; // 本次调头起点，累计编码器计数。
static float yaw_turn_duration_s; // 动作开始时锁定的轨迹时长，s。
static float yaw_turn_elapsed_s; // 已成功发送的轨迹累计时间，s。
static uint32_t yaw_turn_last_ms; // 上次轨迹更新的时间。
volatile float cloud_turn_progress; // 本次轨迹已执行的比例。
volatile float cloud_turn_duration_s; // 本次实际采用的轨迹时长，s。
volatile float cloud_turn_target_deg; // 当前轨迹目标相对机械零点的角度，度。

// 时长只在动作开始时读取，运行中修改留到下次调头。
static float cloud_turn_duration(float duration_s)
{
    float period_s = motor4310_config.control_period_s;
    if (!(duration_s >= 0.004f && duration_s <= 30.0f) ||
        !(period_s > 0.0f && period_s <= duration_s)) { return 0.0f; }
    return duration_s;
}

// 五次曲线将时间进度换成位置进度，两端目标速度和加速度为零。
static float cloud_turn_scurve(float progress)
{
    if (progress <= 0.0f) { return 0.0f; }
    if (progress >= 1.0f) { return 1.0f; }
    return progress * progress * progress *
        (10.0f + progress * (-15.0f + 6.0f * progress));
}
static uint16_t yaw_turn_stable_cycles; // 调头到位的连续控制周期数。
static bool turn_wheel_armed; // 拨轮回到中位后重新允许下降沿。
static bool turn_requested; // 拨轮事件已触发，等待归中完成后执行。
static bool spin_rearm_required; // 运行后持续失去许可，须新的手动使能。
static bool spin_session_started; // 本次手动启动是否已得到过自旋许可。
static bool spin_fault_timing; // 自旋许可丢失计时中。
static uint32_t spin_fault_start_ms; // 本次许可丢失的起始时刻。
static PitchControl_t pitch_control; // Pitch 位控目标状态。
static PID_Controller_t yaw_angle_pid; // Yaw 角度外环 PID。
static PID_Controller_t yaw_rate_pid; // Yaw 角速度内环 PID。

volatile bool cloud_yaw_imu_online; // 上板 IMU 是否已标定且在线。
volatile float cloud_yaw_target_deg; // Yaw 位控累计目标角，单位度。
volatile float cloud_yaw_angle_deg; // 当前 IMU 累计 Yaw 角，单位度。
volatile float cloud_yaw_rate_deg_s; // 当前 IMU Yaw 角速度，单位度每秒。
volatile float cloud_yaw_rate_target_deg_s; // 角度外环给出的角速度目标。
volatile int16_t cloud_yaw_torque_raw; // 角速度内环给出的 4310 原始转矩。
volatile float cloud_yaw_mechanical_error_deg;
volatile float cloud_yaw_mechanical_speed_limit_raw;
volatile bool cloud_yaw_mechanical_gyro_online;
volatile bool cloud_yaw_chassis_rate_online; // 本周期底盘角速度有效状态。
volatile float cloud_yaw_chassis_rate_deg_s; // 底盘角速度，度/s。
volatile float cloud_yaw_chassis_rate_ff_deg_s; // 实际叠加的角速度前馈，度/s。

static void cloud_clear_chassis_rate_ff(void)
{
    cloud_yaw_chassis_rate_online = false;
    cloud_yaw_chassis_rate_deg_s = 0.0f;
    cloud_yaw_chassis_rate_ff_deg_s = 0.0f;
}

static float cloud_mechanical_chassis_rate_ff(void)
{
    float rate, gain = cloud_config.yaw.mechanical.chassis_rate_ff_gain;
    float limit = cloud_config.yaw.mechanical.chassis_rate_ff_limit_deg_s;
    cloud_clear_chassis_rate_ff();
    if (!(gain >= -10.0f && gain <= 10.0f) ||
        !(limit > 0.0f && limit <= 1000.0f) ||
        !Communication_CAN_GetChassisYawRate(&rate) || !isfinite(rate))
    { return 0.0f; }
    cloud_yaw_chassis_rate_online = true;
    cloud_yaw_chassis_rate_deg_s = rate;
    cloud_yaw_chassis_rate_ff_deg_s = fmaxf(-limit, fminf(limit, gain * rate));
    return cloud_yaw_chassis_rate_ff_deg_s;
}
static PID_Controller_t yaw_mechanical_rate_pid; // 机械 Yaw 陀螺仪速度内环。

volatile bool cloud_turnaround_active; // Yaw 正在转向另一个车头方向。
volatile bool cloud_front_reversed; // 逻辑正方向是否为物理车尾。

volatile bool cloud_pitch_imu_online; // Pitch 惯性控制所用的 IMU 是否有效。
volatile float cloud_pitch_target_deg; // Pitch 惯性位置目标，度。
volatile float cloud_pitch_angle_deg; // 当前 IMU Pitch 角，度。
volatile float cloud_pitch_rate_deg_s; // 滤波后的 Pitch 角速度，度/s。
volatile float cloud_pitch_rate_target_deg_s; // Pitch 位置环给出的目标角速度，度/s。
volatile int16_t cloud_pitch_torque_raw; // 叠加重力补偿并限幅后的转矩码。
static PID_Controller_t pitch_angle_pid; // Pitch 惯性位置外环。
static PID_Controller_t pitch_rate_pid; // Pitch 陀螺仪速度内环。
static float pitch_imu_direction; // 当前采用的 IMU 与电机方向关系。
static bool pitch_inertial_mode; // 上周期是否使用 Pitch 惯性控制。

static void cloud_reset_pitch_imu(void)
{
    cloud_pitch_imu_online = false;
    cloud_pitch_rate_target_deg_s = 0.0f;
    cloud_pitch_torque_raw = 0;
    PID_Reset(&pitch_angle_pid);
    PID_Reset(&pitch_rate_pid);
}

static void cloud_reset_home(void)
{
    if (cloud_turnaround_active) { turn_requested = true; }
    cloud_turnaround_active = false;
    yaw_turn_stable_cycles = 0U;
    spin_rearm_required = true;
    spin_session_started = false;
    spin_fault_timing = false;
    cloud_terrace_home_state = CLOUD_TERRACE_HOME_WAIT;
    home_stable_cycles = 0U;
    targets_initialized = false;
    pitch_inertial_mode = false;
    cloud_reset_pitch_imu();
    yaw_mechanical_mode = false;

    yaw_mechanical_target = 0;
    cloud_yaw_mechanical_gyro_online = false;
    PID_Reset(&yaw_mechanical_rate_pid);
    pitch_control.target_counts = 0.0f;
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_rate_pid);
    cloud_yaw_imu_online = false;
    cloud_yaw_target_deg = 0.0f;
    cloud_yaw_angle_deg = 0.0f;
    cloud_yaw_rate_deg_s = 0.0f;
    cloud_clear_chassis_rate_ff();
    cloud_yaw_rate_target_deg_s = 0.0f;
    cloud_yaw_torque_raw = 0;
    home_target[MOTOR4310_PITCH] = 0;
    home_target[MOTOR4310_YAW] = 0;
}

// 按配置初始化惯性闭环和机械 Yaw 速度环，清理前馈并复位开机归中状态。
void CloudTerrace_Init(void)
{
    cloud_clear_chassis_rate_ff();
    PID_Init(&yaw_angle_pid, cloud_config.yaw.inertial.angle.kp, cloud_config.yaw.inertial.angle.ki,
             cloud_config.yaw.inertial.angle.kd, cloud_config.yaw.inertial.angle.integral_limit,
             cloud_config.yaw.inertial.angle.output_limit,
             motor4310_config.control_period_s);
    PID_Init(&yaw_rate_pid, cloud_config.yaw.inertial.rate.kp, cloud_config.yaw.inertial.rate.ki,
             cloud_config.yaw.inertial.rate.kd, cloud_config.yaw.inertial.rate.integral_limit,
             cloud_config.yaw.inertial.rate.output_limit, motor4310_config.control_period_s);
    PID_Init(&pitch_angle_pid, cloud_config.pitch.inertial.angle.kp,
        cloud_config.pitch.inertial.angle.ki, cloud_config.pitch.inertial.angle.kd,
        cloud_config.pitch.inertial.angle.integral_limit,
        cloud_config.pitch.inertial.angle.output_limit, motor4310_config.control_period_s);
    PID_Init(&pitch_rate_pid, cloud_config.pitch.inertial.rate.kp,
        cloud_config.pitch.inertial.rate.ki, cloud_config.pitch.inertial.rate.kd,
        cloud_config.pitch.inertial.rate.integral_limit, cloud_config.pitch.inertial.rate.output_limit,
        motor4310_config.control_period_s);
    PID_Init(&yaw_mechanical_rate_pid, cloud_config.yaw.mechanical.rate.kp,
        cloud_config.yaw.mechanical.rate.ki, cloud_config.yaw.mechanical.rate.kd,
        cloud_config.yaw.mechanical.rate.integral_limit,
        cloud_config.yaw.mechanical.rate.output_limit, motor4310_config.control_period_s);
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

// 以归中点为零角度，重力前馈k*cos(theta)。
static int16_t cloud_pitch_gravity(const Motor4310_Data_t *feedback)
{
    float theta = (float)(feedback->total_angle - home_target[MOTOR4310_PITCH]) *
                  (2.0f * MOTOR4310_PMAX / MOTOR4310_ECD_PER_ROUND);
    float torque_raw = cloud_config.pitch.gravity_k * cosf(theta) *
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
           feedback->speed >= -cloud_config.home.speed_raw_max &&
           feedback->speed <= cloud_config.home.speed_raw_max;
}

static bool cloud_home_step(void)
{
    Motor4310_Data_t pitch, yaw;
    HAL_StatusTypeDef pitch_status, yaw_status;
    int32_t tolerance = Motor4310_PositionToEcd(0.0f,
                                                cloud_config.home.tolerance_deg);

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE) { return true; }
    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &pitch) ||
        !Motor4310_GetFeedback(MOTOR4310_YAW, &yaw)) { return false; }

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_WAIT)
    {
        home_target[MOTOR4310_PITCH] =
            cloud_nearest_home(cloud_config.pitch.home_rad, pitch.total_angle);
        home_target[MOTOR4310_YAW] =
            cloud_nearest_home(cloud_config.yaw.home_rad, yaw.total_angle);
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
        if (++home_stable_cycles >= cloud_config.home.stable_cycles)
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
    return fabsf(relative_deg) < lift_config.safety.yaw_deadzone_deg;
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
    if (wheel > -cloud_config.yaw.turn.wheel_rearm_raw)
    { turn_wheel_armed = true; }
    else if (turn_wheel_armed && wheel <= -cloud_config.yaw.turn.wheel_trigger_raw)
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
    bool reversed;

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        !cloud_yaw_relative_deg(&relative_deg)) { return false; }
    // 当前较近的方向若为车头，则本次转向车尾；反之转向车头。
    reversed = fabsf(relative_deg) < cloud_config.yaw.front_switch_deg;
    yaw_turn_target = cloud_nearest_front_target(reversed,
                                                 yaw.total_angle);
    yaw_turn_start_counts = yaw.total_angle;
    yaw_turn_duration_s = cloud_turn_duration(cloud_config.yaw.turn.duration_s);
    if (yaw_turn_duration_s <= 0.0f) { turn_requested = false; return false; }
    cloud_front_reversed = reversed;
    yaw_turn_elapsed_s = 0.0f;
    yaw_turn_last_ms = HAL_GetTick();
    cloud_turn_progress = 0.0f;
    cloud_turn_duration_s = yaw_turn_duration_s;
    yaw_turn_stable_cycles = 0U;
    turn_requested = false;
    cloud_turnaround_active = true;
    yaw_mechanical_mode = false;

    cloud_yaw_mechanical_gyro_online = false;
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
                                                cloud_config.yaw.turn.tolerance_deg);
    int32_t error;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - yaw_turn_last_ms;
    float next_elapsed, progress, fraction;
    int32_t trajectory_target;
    Motor4310_PidProfile_t turn_pid = motor4310_config.yaw_turn;

    yaw_turn_last_ms = now;
    // 调度长间隔不允许一次跳过大段轨迹；发送失败时保留上一轨迹时间。
    if (elapsed_ms > 20U) { elapsed_ms = 20U; }
    next_elapsed = yaw_turn_elapsed_s + (float)elapsed_ms * 0.001f;
    if (next_elapsed > yaw_turn_duration_s) { next_elapsed = yaw_turn_duration_s; }
    progress = next_elapsed / yaw_turn_duration_s;
    fraction = cloud_turn_scurve(progress);
    trajectory_target = progress >= 1.0f ? yaw_turn_target :
        yaw_turn_start_counts + (int32_t)(fraction *
            (float)(yaw_turn_target - yaw_turn_start_counts));

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        Motor4310_PositionControlWithProfile(
            MOTOR4310_YAW, trajectory_target, 0, &turn_pid, false) != HAL_OK)
    {
        yaw_turn_stable_cycles = 0U;
        return;
    }
    yaw_turn_elapsed_s = next_elapsed;
    cloud_turn_progress = progress;
    cloud_turn_target_deg = (float)(trajectory_target - home_target[MOTOR4310_YAW]) *
        360.0f / MOTOR4310_ECD_PER_ROUND;
    error = yaw_turn_target - yaw.total_angle;
    if (progress >= 1.0f && error >= -tolerance && error <= tolerance &&
        yaw.speed >= -cloud_config.yaw.turn.speed_raw_max &&
        yaw.speed <= cloud_config.yaw.turn.speed_raw_max)
    {
        if (++yaw_turn_stable_cycles >= cloud_config.yaw.turn.stable_cycles)
        {
            cloud_turnaround_active = false;
            Motor4310_ResetControl(MOTOR4310_YAW);
            // 机械模式接管同一个终点，避免到位后按反馈重新选择机械目标。
            yaw_mechanical_target = yaw_turn_target;
            yaw_mechanical_mode = true;
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
        cloud_yaw_mechanical_gyro_online = false;
        // 切回惯性控制时从当前 IMU 朝向重建目标。
        Motor4310_ResetControl(MOTOR4310_YAW);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = false;

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
    cloud_yaw_target_deg += cloud_config.yaw.rc_direction *
        (float)input / CLOUD_RC_MAX_VALUE * cloud_config.yaw.command_rate_deg_s *
        motor4310_config.control_period_s;
    PID_UpdateParameters(&yaw_angle_pid,
        cloud_config.yaw.inertial.angle.kp, cloud_config.yaw.inertial.angle.ki,
        cloud_config.yaw.inertial.angle.kd, cloud_config.yaw.inertial.angle.integral_limit,
        cloud_config.yaw.inertial.angle.output_limit,
        motor4310_config.control_period_s);
    PID_UpdateParameters(&yaw_rate_pid,
        cloud_config.yaw.inertial.rate.kp, cloud_config.yaw.inertial.rate.ki,
        cloud_config.yaw.inertial.rate.kd, cloud_config.yaw.inertial.rate.integral_limit,
        cloud_config.yaw.inertial.rate.output_limit,
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

static void cloud_control_yaw_mechanical(bool use_deadzone, bool force_home)
{
    Motor4310_Data_t yaw;
    GimbalImu_Data_t imu;
    PID_Controller_t *position_pid = &motor4310_position_pids[MOTOR4310_YAW];
    bool gyro_recovered;
    int32_t error, deadzone_counts, control_target;
    float remaining_deg, brake_limit, brake_gain, speed_limit, speed_command;
    float command_scale, direction, torque;

    if (!yaw_mechanical_mode || (force_home && cloud_front_reversed))
    {
        if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw))
        {
            cloud_yaw_torque_raw = 0;
            return;
        }

        // 前/后方向只改变目标，不重新定义开机校准的机械零点。
        cloud_front_reversed = !force_home && fabsf(cloud_wrap_yaw_deg(
            (float)(yaw.total_angle - home_target[MOTOR4310_YAW]) *
            360.0f / MOTOR4310_ECD_PER_ROUND)) >= cloud_config.yaw.front_switch_deg;
        yaw_mechanical_target = cloud_nearest_front_target(
            cloud_front_reversed, yaw.total_angle);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        Motor4310_ResetControl(MOTOR4310_YAW);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = true;
        cloud_yaw_mechanical_gyro_online = false;
        PID_Reset(&yaw_mechanical_rate_pid);

    }

    cloud_yaw_rate_target_deg_s = 0.0f;
    cloud_yaw_torque_raw = 0;
    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw)) { return; }
    if (!GimbalImu_Get(&imu))
    {
        // 陀螺仪不可用就停闭环；保留机械目标，恢复后继续回正。
        cloud_yaw_mechanical_gyro_online = false;
        Motor4310_ResetControl(MOTOR4310_YAW);
        PID_Reset(&yaw_mechanical_rate_pid);
        (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW, 0);
        return;
    }
    gyro_recovered = !cloud_yaw_mechanical_gyro_online;
    cloud_yaw_mechanical_gyro_online = true;
    error = yaw_mechanical_target - yaw.total_angle;
    cloud_yaw_mechanical_error_deg = (float)error * 360.0f / MOTOR4310_ECD_PER_ROUND;
    control_target = yaw_mechanical_target;
    if (use_deadzone)
    {
        float deadzone_deg = cloud_config.yaw.mechanical.deadzone_deg;
        if (!(deadzone_deg >= 0.0f && deadzone_deg <= 10.0f)) { deadzone_deg = 1.0f; }
        deadzone_counts = Motor4310_PositionToEcd(0.0f, deadzone_deg);
        // 连续软死区：边界外只纠正超出死区的误差；内部速度环继续制动。
        if (error > deadzone_counts) { control_target -= deadzone_counts; }
        else if (error < -deadzone_counts) { control_target += deadzone_counts; }
        else { control_target = yaw.total_angle; }
    }
    // 编码器位置外环保持原参数；速度码只是目标指令，随后换算为 °/s。
    speed_limit = fabsf(motor4310_config.yaw_hold.position.output_limit);
    brake_gain = cloud_config.yaw.mechanical.brake_speed_at_1deg_raw;
    if (brake_gain > 0.0f)
    {
        remaining_deg = fabsf((float)(control_target - yaw.total_angle)) *
            360.0f / MOTOR4310_ECD_PER_ROUND;
        brake_limit = brake_gain * sqrtf(remaining_deg);
        if (speed_limit > brake_limit) { speed_limit = brake_limit; }
    }
    cloud_yaw_mechanical_speed_limit_raw = speed_limit;
    PID_UpdateParameters(position_pid, motor4310_config.yaw_hold.position.kp,
        motor4310_config.yaw_hold.position.ki, motor4310_config.yaw_hold.position.kd,
        motor4310_config.yaw_hold.position.integral_limit, speed_limit,
        motor4310_config.control_period_s);
    if (gyro_recovered)
    {
        // 恢复时避免位置微分把当前误差当成突变。
        PID_Reset(position_pid);
        position_pid->LastError = (float)(control_target - yaw.total_angle);
        PID_Reset(&yaw_mechanical_rate_pid);
    }
    speed_command = PID_Calc(position_pid, (float)control_target, (float)yaw.total_angle);
    command_scale = cloud_config.yaw.mechanical.command_deg_s_per_raw;
    if (!(command_scale > 0.0f && command_scale <= 100.0f)) { command_scale = 1.0f; }
    direction = cloud_config.yaw.mechanical.gyro_direction < 0.0f ? -1.0f : 1.0f;
    // 相对位置保持时，惯性系目标速度需要包含底盘自身转速。
    cloud_yaw_rate_target_deg_s = speed_command * command_scale +
        cloud_mechanical_chassis_rate_ff();
    cloud_yaw_rate_deg_s = direction * imu.yaw_rate_deg_s;
    PID_UpdateParameters(&yaw_mechanical_rate_pid,
        cloud_config.yaw.mechanical.rate.kp, cloud_config.yaw.mechanical.rate.ki,
        cloud_config.yaw.mechanical.rate.kd, cloud_config.yaw.mechanical.rate.integral_limit,
        fminf(2047.0f, fabsf(cloud_config.yaw.mechanical.rate.output_limit)),
        motor4310_config.control_period_s);
    if (gyro_recovered)
    {
        yaw_mechanical_rate_pid.LastError =
            cloud_yaw_rate_target_deg_s - cloud_yaw_rate_deg_s;
    }
    // 内环使用上板陀螺仪；死区内取消回正速度，底盘随动前馈仍保留。
    torque = PID_Calc(&yaw_mechanical_rate_pid, cloud_yaw_rate_target_deg_s,
                     cloud_yaw_rate_deg_s);
    cloud_yaw_torque_raw = (int16_t)torque;
    (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW, cloud_yaw_torque_raw);
}

static void cloud_send_yaw_angle(bool allow_turn, bool allow_spin,
                                 bool spin_selected, bool bottom_mode_blocked)
{
    float relative_deg = 0.0f;
    bool angle_valid = cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE &&
        Motor4310_OnlineCheck(MOTOR4310_YAW) &&
        cloud_yaw_relative_deg(&relative_deg);

    // 角度不可用时仍发安全模式状态；下板的自旋档位不依赖角度有效位。
    if (!angle_valid) { allow_turn = false; allow_spin = false; }
    (void)Communication_CAN_SendYawState(relative_deg,
                                         cloud_turnaround_active, allow_turn,
                                         allow_spin, spin_selected,
                                         angle_valid, bottom_mode_blocked);
}

// 外环使用融合姿态；内环使用陀螺仪换算的俯仰角导数，单位均为度。
static void cloud_control_pitch_imu(int16_t input,
    const Motor4310_Data_t *feedback, int32_t lower, int32_t upper, int16_t gravity)
{
    GimbalImu_Data_t imu;
    const float rad_to_deg = 57.2957795131f;
    float direction = cloud_config.pitch.imu_direction < 0.0f ? -1.0f : 1.0f;
    float angle, rate, roll, alpha, lead, lower_deg, upper_deg, torque, limit;
    if (!GimbalImu_Get(&imu))
    {
        cloud_reset_pitch_imu();
        (void)Motor4310_SetTorqueRawMotor(MOTOR4310_PITCH, 0);
        return;
    }
    angle = direction * imu.pitch_deg;
    roll = imu.roll_deg / rad_to_deg;
    rate = direction * (cosf(roll) * imu.gyro_rad_s[1] -
                       sinf(roll) * imu.gyro_rad_s[2]) * rad_to_deg;
    if (!cloud_pitch_imu_online || direction != pitch_imu_direction)
    {
        // 模式切入、IMU 恢复或方向修改，均从实际姿态接管。
        Motor4310_ResetControl(MOTOR4310_PITCH);
        cloud_reset_pitch_imu();
        cloud_pitch_target_deg = angle;
        cloud_pitch_rate_deg_s = rate;
        pitch_imu_direction = direction;
    }
    cloud_pitch_imu_online = true;
    cloud_pitch_angle_deg = angle;
    alpha = cloud_config.pitch.rate_filter_alpha;
    if (!(alpha > 0.0f && alpha <= 1.0f)) { alpha = 1.0f; }
    cloud_pitch_rate_deg_s += alpha * (rate - cloud_pitch_rate_deg_s);
    if (input > cloud_config.rc_speed_enter || input < -cloud_config.rc_speed_enter)
    {
        cloud_pitch_target_deg += (float)input / CLOUD_RC_MAX_VALUE *
            cloud_config.pitch.command_rate_deg_s * motor4310_config.control_period_s;
        lead = fmaxf(0.0f, cloud_config.pitch.target_lead_deg);
        cloud_pitch_target_deg = fmaxf(angle - lead,
            fminf(angle + lead, cloud_pitch_target_deg));
    }
    // 用实时编码器余量约束惯性目标，车体倾斜时机械限位仍有效。
    lower_deg = angle + (float)(lower - feedback->total_angle) *
        360.0f / MOTOR4310_ECD_PER_ROUND;
    upper_deg = angle + (float)(upper - feedback->total_angle) *
        360.0f / MOTOR4310_ECD_PER_ROUND;
    cloud_pitch_target_deg = fmaxf(lower_deg, fminf(upper_deg, cloud_pitch_target_deg));
    PID_UpdateParameters(&pitch_angle_pid, cloud_config.pitch.inertial.angle.kp,
        cloud_config.pitch.inertial.angle.ki, cloud_config.pitch.inertial.angle.kd,
        cloud_config.pitch.inertial.angle.integral_limit, cloud_config.pitch.inertial.angle.output_limit,
        motor4310_config.control_period_s);
    PID_UpdateParameters(&pitch_rate_pid, cloud_config.pitch.inertial.rate.kp,
        cloud_config.pitch.inertial.rate.ki, cloud_config.pitch.inertial.rate.kd,
        cloud_config.pitch.inertial.rate.integral_limit, cloud_config.pitch.inertial.rate.output_limit,
        motor4310_config.control_period_s);
    cloud_pitch_rate_target_deg_s = PID_Calc(&pitch_angle_pid, cloud_pitch_target_deg, angle);
    // 到达限位后拦截向外的目标速度，含积分产生的速度。
    if ((feedback->total_angle <= lower && cloud_pitch_rate_target_deg_s < 0.0f) ||
        (feedback->total_angle >= upper && cloud_pitch_rate_target_deg_s > 0.0f))
    { cloud_pitch_rate_target_deg_s = 0.0f; }
    // 速度环纠偏与机械角重力补偿相加，再统一限制转矩。
    torque = PID_Calc(&pitch_rate_pid, cloud_pitch_rate_target_deg_s,
                     cloud_pitch_rate_deg_s) + (float)gravity;
    limit = fminf(2047.0f, fabsf(cloud_config.pitch.inertial.rate.output_limit));
    torque = fmaxf(-limit, fminf(limit, torque));
    cloud_pitch_torque_raw = (int16_t)torque;
    (void)Motor4310_SetTorqueRawMotor(MOTOR4310_PITCH, cloud_pitch_torque_raw);
}

static void cloud_control_pitch(int16_t input, bool use_imu)
{
    Motor4310_Data_t feedback;
    int32_t lower, upper;
    float lead_counts;
    int16_t gravity;

    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &feedback)) { return; }
    lower = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, cloud_config.pitch.min_deg);
    if (lift_pitch_nonnegative_required)
    {
        int32_t lift_lower = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f,
                cloud_config.pitch.lift_clearance_deg > 0.0f ?
                cloud_config.pitch.lift_clearance_deg : 0.0f);
        if (lower < lift_lower) { lower = lift_lower; }
    }
    upper = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, cloud_config.pitch.max_deg);
    gravity = cloud_pitch_gravity(&feedback);
    if (use_imu)
    {
        pitch_inertial_mode = true;
        cloud_control_pitch_imu(input, &feedback, lower, upper, gravity);
        return;
    }
    if (pitch_inertial_mode)
    {
        pitch_inertial_mode = false;
        cloud_reset_pitch_imu();
        Motor4310_ResetControl(MOTOR4310_PITCH);
        pitch_control.target_counts = (float)feedback.total_angle;
    }
    if (input > cloud_config.rc_speed_enter ||
        input < -cloud_config.rc_speed_enter)
    {
        // 遥控只积分位置目标，不切换到底层速度控制。
        pitch_control.target_counts += (float)input / CLOUD_RC_MAX_VALUE *
            cloud_config.pitch.command_rate_deg_s *
            motor4310_config.control_period_s *
            (MOTOR4310_ECD_PER_ROUND / 360.0f);
        lead_counts = cloud_config.pitch.target_lead_deg *
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

// 首次启动可等待许可；已运行后持续失去许可才锁存重新布防。
static void cloud_spin_rearm_update(const RemoteState_t *remote, bool permitted)
{
    if (remote->mode.chassis != REMOTE_MODE_SPIN || !remote->safety.spin_enabled)
    {
        spin_rearm_required = false;
        spin_session_started = false;
        spin_fault_timing = false;
    }
    else if (permitted) { spin_fault_timing = false; }
    else if (spin_session_started)
    {
        if (!spin_fault_timing)
        {
            spin_fault_timing = true;
            spin_fault_start_ms = HAL_GetTick();
        }
        else if ((uint32_t)(HAL_GetTick() - spin_fault_start_ms) >=
                     cloud_config.spin.fault_rearm_ms)
        { spin_rearm_required = true; }
    }
}

void CloudTerrace_Update(const RemoteState_t *remote_snapshot)
{
    cloud_clear_chassis_rate_ff();
    RemoteState_t remote;
    LiftSafetyState_t safety;
    HAL_StatusTypeDef pitch_enable, yaw_enable;
    bool turn_blocked;
    bool mode_permitted;
    bool spin_permitted;
    bool bottom_mode_blocked;
    bool yaw_home_required;
    bool safety_valid;
    bool pitch_use_imu;

    Motor4310_Heartbeat();
    if (remote_snapshot == NULL) { return; }
    remote = *remote_snapshot;
    safety_valid = LiftControl_SafetyGet(&safety);
    mode_permitted = safety_valid && safety.special_allowed;
    bottom_mode_blocked = safety.bottom_mode_blocked;
    yaw_home_required = safety.yaw_home_required;
    if (remote.mode.chassis != REMOTE_MODE_FOLLOW &&
        remote.mode.chassis != REMOTE_MODE_MECHANICAL &&
        remote.mode.chassis != REMOTE_MODE_SPIN)
    {
        // 遥控断联或档位无效：取消调头并让两轴失能。
        cloud_reset_home();
        turn_requested = false;
        turn_wheel_armed = false;
        cloud_front_reversed = false;
        cloud_send_yaw_angle(false, false, false, bottom_mode_blocked);
        (void)Motor4310_DisableMotor(MOTOR4310_PITCH);
        (void)Motor4310_DisableMotor(MOTOR4310_YAW);
        return;
    }

    cloud_spin_rearm_update(&remote, safety.spin_allowed);
    spin_permitted = remote.mode.chassis == REMOTE_MODE_SPIN &&
                     safety.spin_allowed && !spin_rearm_required &&
                     !cloud_turnaround_active && !yaw_home_required;
    if (spin_permitted) { spin_session_started = true; }
    turn_blocked = (remote.mode.chassis == REMOTE_MODE_SPIN &&
                    remote.safety.spin_enabled) ||
                   !mode_permitted || yaw_home_required;
    if (turn_blocked && cloud_turnaround_active)
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
        cloud_send_yaw_angle(false, false,
                             remote.mode.chassis == REMOTE_MODE_SPIN,
                             bottom_mode_blocked);
        return;
    }
    if (!cloud_home_step())
    {
        cloud_send_yaw_angle(false, false,
                             remote.mode.chassis == REMOTE_MODE_SPIN,
                             bottom_mode_blocked);
        return;
    }
    if (!targets_initialized)
    {
        pitch_control.target_counts = (float)home_target[MOTOR4310_PITCH];
        targets_initialized = true;
    }

    pitch_use_imu = lift_calibrated && !bottom_mode_blocked && !yaw_home_required &&
        (remote.mode.chassis == REMOTE_MODE_FOLLOW || remote.mode.chassis == REMOTE_MODE_SPIN);
    if (turn_requested && !cloud_turnaround_active && !turn_blocked)
    { (void)cloud_turn_start(); }
    if (cloud_turnaround_active)
    {
        // 下板已由同一拨轮边沿先行停车；此时只让云台轴执行控制。
        cloud_turn_step();
        cloud_control_pitch(remote.input.channel[1], pitch_use_imu);
        cloud_send_yaw_angle(!turn_blocked, false,
                             remote.mode.chassis == REMOTE_MODE_SPIN,
                             bottom_mode_blocked);
        return;
    }

    if (!lift_calibrated || bottom_mode_blocked || yaw_home_required ||
        remote.mode.chassis == REMOTE_MODE_MECHANICAL)
    {
        // 升降指令或低位联锁强制回机械正方向，再由升降任务确认稳定。
        cloud_control_yaw_mechanical(
            bottom_mode_blocked || yaw_home_required ||
                remote.mode.chassis == REMOTE_MODE_MECHANICAL,
            bottom_mode_blocked || yaw_home_required);
    }
    else
    {
        // 跟随及小陀螺模式使用惯性系 Yaw 控制。
        cloud_control_yaw(-remote.input.channel[0]);
    }
    cloud_control_pitch(remote.input.channel[1], pitch_use_imu);
    cloud_send_yaw_angle(!turn_blocked, spin_permitted,
                         remote.mode.chassis == REMOTE_MODE_SPIN,
                         bottom_mode_blocked);
}
