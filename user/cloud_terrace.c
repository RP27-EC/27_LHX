#include "cloud_terrace.h"
#include "communication.h"
#include "imu.h"
#include "motor4310.h"
#include "PID.h"
#include "parameter.h"
#include "remote_state.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool speed_mode;   /* true 为遥控速度输入，false 为零速角度保持。 */
    int32_t hold_angle;/* 速度输入归零瞬间锁定的 Pitch 累计编码器角度。 */
} PitchControl_t;

volatile CloudTerrace_HomeState_t cloud_terrace_home_state =
    CLOUD_TERRACE_HOME_WAIT; /* 当前归中流程状态，供控制与调试观察。 */
static int32_t home_target[MOTOR4310_COUNT]; /* 两轴归中时的机械位置目标。 */
static uint16_t home_stable_cycles;          /* 两轴连续处于归中误差内的周期数。 */
static bool targets_initialized;            /* 常规控制目标是否已从反馈值初始化。 */
static bool yaw_mechanical_mode;             /* 是否处于 Yaw 始终朝底盘正前方模式。 */
static int32_t yaw_mechanical_target;        /* 机械模式选定的物理前/后归中目标。 */
static int32_t yaw_turn_target;              /* 本次调头的 Yaw 累计编码器目标。 */
static uint16_t yaw_turn_stable_cycles;      /* 调头到位的连续控制周期数。 */
static bool turn_wheel_armed;                /* 拨轮回到中位后重新允许下降沿。 */
static bool turn_requested;                  /* 拨轮事件已触发，等待归中完成后执行。 */
static PitchControl_t pitch_control;         /* Pitch 速度/保持模式内部状态。 */
static PID_Controller_t yaw_angle_pid;       /* Yaw 角度外环 PID。 */
static PID_Controller_t yaw_rate_pid;        /* Yaw 角速度内环 PID。 */

volatile bool cloud_yaw_imu_online;          /* 上板 IMU 是否已标定且在线。 */
volatile float cloud_yaw_target_deg;         /* Yaw 位控累计目标角，单位度。 */
volatile float cloud_yaw_angle_deg;          /* 当前 IMU 累计 Yaw 角，单位度。 */
volatile float cloud_yaw_rate_deg_s;         /* 当前 IMU Yaw 角速度，单位度每秒。 */
volatile float cloud_yaw_rate_target_deg_s;  /* 角度外环给出的角速度目标。 */
volatile int16_t cloud_yaw_torque_raw;       /* 角速度内环给出的 4310 原始转矩。 */
volatile bool cloud_turnaround_active;      /* Yaw 正在转向另一个车头方向。 */
volatile bool cloud_front_reversed;         /* 逻辑正方向是否为物理车尾。 */

static void cloud_reset_home(void)
{
    if (cloud_turnaround_active) { turn_requested = true; }
    cloud_turnaround_active = false;
    yaw_turn_stable_cycles = 0U;
    cloud_terrace_home_state = CLOUD_TERRACE_HOME_WAIT;
    home_stable_cycles = 0U;
    targets_initialized = false;
    yaw_mechanical_mode = false;
    yaw_mechanical_target = 0;
    pitch_control.speed_mode = false;
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
    PID_Init(&yaw_angle_pid, CLOUD_YAW_ANGLE_KP, CLOUD_YAW_ANGLE_KI,
             CLOUD_YAW_ANGLE_KD, CLOUD_YAW_ANGLE_INTEGRAL_LIMIT,
             CLOUD_YAW_RATE_TARGET_LIMIT_DEG_S,
             MOTOR4310_CONTROL_PERIOD_S);
    PID_Init(&yaw_rate_pid, CLOUD_YAW_RATE_KP, CLOUD_YAW_RATE_KI,
             CLOUD_YAW_RATE_KD, CLOUD_YAW_RATE_INTEGRAL_LIMIT,
             CLOUD_YAW_TORQUE_LIMIT_RAW, MOTOR4310_CONTROL_PERIOD_S);
    pitch_control.hold_angle = 0;
    cloud_reset_home();
}

/* 将 [-PI, PI] 电机角换成累计编码器计数，并选择最近的一圈。 */
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

/* 在物理车头 0° 与车尾 180° 中，求距当前电机角最近的等效累计目标。 */
static int32_t cloud_nearest_front_target(bool reversed, int32_t current)
{
    const int32_t period = (int32_t)(MOTOR4310_ECD_PER_ROUND + 0.5f);
    int32_t target = cloud_nearest_home(CLOUD_YAW_HOME_RAD, current);
    if (reversed) { target += period / 2; }
    while (target - current > period / 2) { target -= period; }
    while (target - current < -(period / 2)) { target += period; }
    return target;
}

/* Pitch 重力补偿采用余弦转矩曲线；驱动接收原始转矩码前馈。 */
static int16_t cloud_pitch_gravity(uint16_t encoder_angle)
{
    float motor_rad = (float)encoder_angle *
                      (2.0f * MOTOR4310_PMAX / 65535.0f) - MOTOR4310_PMAX;
    float gravity_nm = cosf(motor_rad - CLOUD_PITCH_GRAVITY_CENTER_RAD) *
                       CLOUD_PITCH_GRAVITY_K + CLOUD_PITCH_GRAVITY_B;
    return (int16_t)(gravity_nm * CLOUD_PITCH_GRAVITY_SCALE *
                     (4095.0f / (2.0f * CLOUD_MOTOR_TORQUE_MAX_NM)));
}

static bool cloud_at_home(const Motor4310_Data_t *feedback, int32_t target,
                          int32_t tolerance)
{
    int32_t error = target - feedback->total_angle;
    return error >= -tolerance && error <= tolerance &&
           feedback->speed >= -CLOUD_HOME_SPEED_RAW_MAX &&
           feedback->speed <= CLOUD_HOME_SPEED_RAW_MAX;
}

static bool cloud_home_step(void)
{
    Motor4310_Data_t pitch, yaw;
    HAL_StatusTypeDef pitch_status, yaw_status;
    int32_t tolerance = Motor4310_PositionToEcd(0.0f,
                                                CLOUD_HOME_TOLERANCE_DEG);

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_DONE) { return true; }
    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &pitch) ||
        !Motor4310_GetFeedback(MOTOR4310_YAW, &yaw)) { return false; }

    if (cloud_terrace_home_state == CLOUD_TERRACE_HOME_WAIT)
    {
        home_target[MOTOR4310_PITCH] =
            cloud_nearest_home(CLOUD_PITCH_HOME_RAD, pitch.total_angle);
        home_target[MOTOR4310_YAW] =
            cloud_nearest_home(CLOUD_YAW_HOME_RAD, yaw.total_angle);
        Motor4310_ResetControl(MOTOR4310_PITCH);
        Motor4310_ResetControl(MOTOR4310_YAW);
        home_stable_cycles = 0U;
        cloud_terrace_home_state = CLOUD_TERRACE_HOME_MOVING;
    }

    pitch_status = Motor4310_PositionControlWithFeedforward(
        MOTOR4310_PITCH, home_target[MOTOR4310_PITCH],
        cloud_pitch_gravity(pitch.angle));
    yaw_status = Motor4310_PositionControlMotor(MOTOR4310_YAW,
                                                home_target[MOTOR4310_YAW]);
    if (pitch_status != HAL_OK || yaw_status != HAL_OK)
    {
        home_stable_cycles = 0U;
        return false;
    }
    if (cloud_at_home(&pitch, home_target[MOTOR4310_PITCH], tolerance) &&
        cloud_at_home(&yaw, home_target[MOTOR4310_YAW], tolerance))
    {
        if (++home_stable_cycles >= CLOUD_HOME_STABLE_CYCLES)
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

static void cloud_turn_wheel_update(int16_t wheel)
{
    if (wheel > -CLOUD_TURN_WHEEL_REARM_RAW)
    { turn_wheel_armed = true; }
    else if (turn_wheel_armed && wheel <= -CLOUD_TURN_WHEEL_TRIGGER_RAW)
    {
        turn_wheel_armed = false;
        if (!cloud_turnaround_active) { turn_requested = true; }
    }
}

static bool cloud_turn_start(void)
{
    Motor4310_Data_t yaw;
    float relative_deg;

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        !cloud_yaw_relative_deg(&relative_deg)) { return false; }
    /* 当前较近的方向若为车头，则本次转向车尾；反之转向车头。 */
    cloud_front_reversed = fabsf(relative_deg) < CLOUD_FRONT_SWITCH_DEG;
    yaw_turn_target = cloud_nearest_front_target(cloud_front_reversed,
                                                 yaw.total_angle);
    yaw_turn_stable_cycles = 0U;
    turn_requested = false;
    cloud_turnaround_active = true;
    yaw_mechanical_mode = false;
    cloud_yaw_imu_online = false; /* 完成后惯性控制从实际朝向重新接管。 */
    Motor4310_ResetControl(MOTOR4310_YAW);
    PID_Reset(&yaw_angle_pid);
    PID_Reset(&yaw_rate_pid);
    return true;
}

static void cloud_turn_step(void)
{
    Motor4310_Data_t yaw;
    int32_t tolerance = Motor4310_PositionToEcd(0.0f,
                                                CLOUD_TURN_TOLERANCE_DEG);
    int32_t error;

    if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw) ||
        Motor4310_PositionControlMotor(MOTOR4310_YAW,
                                       yaw_turn_target) != HAL_OK)
    {
        yaw_turn_stable_cycles = 0U;
        return;
    }
    error = yaw_turn_target - yaw.total_angle;
    if (error >= -tolerance && error <= tolerance &&
        yaw.speed >= -CLOUD_TURN_SPEED_RAW_MAX &&
        yaw.speed <= CLOUD_TURN_SPEED_RAW_MAX)
    {
        if (++yaw_turn_stable_cycles >= CLOUD_TURN_STABLE_CYCLES)
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
        /* 从机械位控切回惯性系控制时清空两套控制器，随后从当前
         * IMU朝向重新建立目标，避免模式切换产生转矩突跳。 */
        Motor4310_ResetControl(MOTOR4310_YAW);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = false;
    }

    if (input >= -CLOUD_RC_SPEED_ENTER && input <= CLOUD_RC_SPEED_ENTER)
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
        /* IMU 恢复时从当前朝向重新接管，避免追赶旧目标突跳。 */
        cloud_yaw_target_deg = imu.yaw_total_deg;
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
    }
    cloud_yaw_imu_online = true;
    cloud_yaw_angle_deg = imu.yaw_total_deg;
    cloud_yaw_rate_deg_s = imu.yaw_rate_deg_s;
    /* 摇杆改变惯性系角度目标；松杆后目标不变，云台稳向。 */
    cloud_yaw_target_deg += CLOUD_YAW_RC_DIRECTION *
        (float)input / CLOUD_RC_MAX_VALUE * CLOUD_YAW_COMMAND_RATE_DEG_S *
        MOTOR4310_CONTROL_PERIOD_S;
    cloud_yaw_rate_target_deg_s = PID_Calc(
        &yaw_angle_pid, cloud_yaw_target_deg, imu.yaw_total_deg);
    torque = PID_Calc(&yaw_rate_pid, cloud_yaw_rate_target_deg_s,
                      imu.yaw_rate_deg_s);
    cloud_yaw_torque_raw = (int16_t)torque;
    (void)Motor4310_SetTorqueRawMotor(MOTOR4310_YAW,
                                      cloud_yaw_torque_raw);
}

static void cloud_control_yaw_mechanical(void)
{
    if (!yaw_mechanical_mode)
    {
        Motor4310_Data_t yaw;

        if (!Motor4310_GetFeedback(MOTOR4310_YAW, &yaw))
        {
            cloud_yaw_torque_raw = 0;
            return;
        }

        /* 机械模式也使用最近的物理前/后方向，避免小陀螺后多转。 */
        home_target[MOTOR4310_YAW] =
            cloud_nearest_home(CLOUD_YAW_HOME_RAD, yaw.total_angle);
        cloud_front_reversed = fabsf(cloud_wrap_yaw_deg(
            (float)(yaw.total_angle - home_target[MOTOR4310_YAW]) *
            360.0f / MOTOR4310_ECD_PER_ROUND)) >= CLOUD_FRONT_SWITCH_DEG;
        yaw_mechanical_target = cloud_nearest_front_target(
            cloud_front_reversed, yaw.total_angle);
        PID_Reset(&yaw_angle_pid);
        PID_Reset(&yaw_rate_pid);
        Motor4310_ResetControl(MOTOR4310_YAW);
        cloud_yaw_imu_online = false;
        yaw_mechanical_mode = true;
    }

    cloud_yaw_rate_target_deg_s = 0.0f;
    cloud_yaw_torque_raw = 0;
    (void)Motor4310_PositionControlMotor(MOTOR4310_YAW,
                                         yaw_mechanical_target);
}

static void cloud_send_yaw_angle(void)
{
    float relative_deg;

    if (!cloud_yaw_relative_deg(&relative_deg)) { return; }
    /* 传机械归中后的相对车头角，而非电机编码器原始零点。 */
    (void)Communication_CAN_SendYawState(relative_deg,
                                         cloud_turnaround_active);
}

static void cloud_control_pitch(int16_t input)
{
    Motor4310_Data_t feedback;
    int32_t lower, upper, remaining = 0, boundary = 0;
    int32_t slow_counts;
    int16_t speed, threshold, gravity;
    bool moving, at_boundary = false;

    if (!Motor4310_GetFeedback(MOTOR4310_PITCH, &feedback)) { return; }
    lower = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, CLOUD_PITCH_MIN_DEG);
    upper = home_target[MOTOR4310_PITCH] +
            Motor4310_PositionToEcd(0.0f, CLOUD_PITCH_MAX_DEG);
    gravity = cloud_pitch_gravity(feedback.angle);
    threshold = pitch_control.speed_mode ? CLOUD_RC_SPEED_EXIT :
                                           CLOUD_RC_SPEED_ENTER;
    moving = input > threshold || input < -threshold;
    if (input < 0)
    {
        remaining = feedback.total_angle - lower;
        if (remaining <= 0)
        { moving = false; at_boundary = true; boundary = lower; }
    }
    else if (input > 0)
    {
        remaining = upper - feedback.total_angle;
        if (remaining <= 0)
        { moving = false; at_boundary = true; boundary = upper; }
    }

    if (moving)
    {
        if (!pitch_control.speed_mode)
        {
            Motor4310_ResetControl(MOTOR4310_PITCH);
            pitch_control.speed_mode = true;
        }
        speed = (int16_t)((float)input / CLOUD_RC_MAX_VALUE *
                          CLOUD_PITCH_MAX_SPEED_RAW);
        /* 接近参数设定的机械边界时减速，边界处转为位置保持。 */
        slow_counts = Motor4310_PositionToEcd(0.0f,
                                               CLOUD_PITCH_LIMIT_SLOW_DEG);
        if (remaining < slow_counts)
        { speed = (int16_t)((int32_t)speed * remaining / slow_counts); }
        (void)Motor4310_SpeedControlWithFeedforward(MOTOR4310_PITCH,
                                                    speed, gravity);
    }
    else
    {
        if (pitch_control.speed_mode)
        {
            /* 松杆时锁存实际角度，不回到归中点。 */
            pitch_control.hold_angle = feedback.total_angle;
            Motor4310_ResetControl(MOTOR4310_PITCH);
            pitch_control.speed_mode = false;
        }
        if (at_boundary) { pitch_control.hold_angle = boundary; }
        if (pitch_control.hold_angle < lower) { pitch_control.hold_angle = lower; }
        if (pitch_control.hold_angle > upper) { pitch_control.hold_angle = upper; }
        (void)Motor4310_PositionControlWithFeedforward(
            MOTOR4310_PITCH, pitch_control.hold_angle, gravity);
    }
}

void CloudTerrace_Update(void)
{
    RemoteState_t remote;
    HAL_StatusTypeDef pitch_enable, yaw_enable;

    Motor4310_Heartbeat();
    RemoteState_Get(&remote);
    if (remote.mode != REMOTE_MODE_FOLLOW &&
        remote.mode != REMOTE_MODE_MECHANICAL &&
        remote.mode != REMOTE_MODE_SPIN)
    {
        /* 遥控断联或档位无效：取消调头并让两轴失能。 */
        cloud_reset_home();
        turn_requested = false;
        turn_wheel_armed = false;
        cloud_front_reversed = false;
        (void)Motor4310_DisableMotor(MOTOR4310_PITCH);
        (void)Motor4310_DisableMotor(MOTOR4310_YAW);
        return;
    }

    cloud_turn_wheel_update(remote.channel[4]);

    pitch_enable = Motor4310_EnableMotor(MOTOR4310_PITCH);
    yaw_enable = Motor4310_EnableMotor(MOTOR4310_YAW);
    if (pitch_enable != HAL_OK || yaw_enable != HAL_OK ||
        !Motor4310_AllOnline())
    {
        /* 任意一轴掉线即停止两轴闭环，恢复后重新归中。 */
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
        pitch_control.hold_angle = home_target[MOTOR4310_PITCH];
        pitch_control.speed_mode = false;
        targets_initialized = true;
    }

    if (turn_requested && !cloud_turnaround_active)
    { (void)cloud_turn_start(); }
    if (cloud_turnaround_active)
    {
        /* 下板已由同一拨轮边沿先行停车；此时只让云台轴执行控制。 */
        cloud_turn_step();
        cloud_control_pitch(remote.channel[1]);
        cloud_send_yaw_angle();
        return;
    }

    if (remote.mode == REMOTE_MODE_MECHANICAL)
    {
        cloud_control_yaw_mechanical();
    }
    else
    {
        /* s0上档和小陀螺组合均使用惯性系Yaw；小陀螺中底盘
         * 自转，云台仍可由摇杆改变指向并在松杆后稳向。 */
        cloud_control_yaw(-remote.channel[0]);
    }
    cloud_control_pitch(remote.channel[1]);
    cloud_send_yaw_angle();
}
