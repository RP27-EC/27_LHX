#include "cloud_terrace.h"
#include "communication.h"
#include "motor4310.h"
#include "parameter.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool speed_mode;
    int32_t hold_angle;
} PitchControl_t;

volatile CloudTerrace_HomeState_t cloud_terrace_home_state =
    CLOUD_TERRACE_HOME_WAIT;
static int32_t home_target[MOTOR4310_COUNT];
static uint16_t home_stable_cycles;
static bool targets_initialized;
static PitchControl_t pitch_control;

static void cloud_reset_home(void)
{
    cloud_terrace_home_state = CLOUD_TERRACE_HOME_WAIT;
    home_stable_cycles = 0U;
    targets_initialized = false;
    pitch_control.speed_mode = false;
    home_target[MOTOR4310_PITCH] = 0;
    home_target[MOTOR4310_YAW] = 0;
}

void CloudTerrace_Init(void)
{
    pitch_control.hold_angle = 0;
    cloud_reset_home();
}

/* 模板的 [-PI, PI] 电机角换成驱动累计编码器角，选择最近的一圈。 */
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

/* 仅 Pitch 使用模板重力曲线；电机驱动只接收原始转矩前馈。 */
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

static void cloud_control_yaw(int16_t input)
{
    int16_t speed;
    /* Yaw 始终为速度环，摇杆回中时仅给零速，不锁住角度。 */
    if (input >= -CLOUD_RC_SPEED_ENTER && input <= CLOUD_RC_SPEED_ENTER)
    { input = 0; }
    speed = (int16_t)((float)input / CLOUD_RC_MAX_VALUE *
                      CLOUD_YAW_MAX_SPEED_RAW);
    (void)Motor4310_SpeedControlMotor(MOTOR4310_YAW, speed);
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
    Communication_RcControl_t rc;
    HAL_StatusTypeDef pitch_enable, yaw_enable;

    Motor4310_Heartbeat();
    if (!Communication_RC_Get(&rc) || rc.rc.s[0] != COMM_RC_SW_UP)
    {
        /* 遥控断联或退出云台模式：两轴失能，驱动定期重发失能帧。 */
        cloud_reset_home();
        (void)Motor4310_DisableMotor(MOTOR4310_PITCH);
        (void)Motor4310_DisableMotor(MOTOR4310_YAW);
        return;
    }

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
        Motor4310_ResetControl(MOTOR4310_YAW);
        targets_initialized = true;
    }
    cloud_control_yaw(rc.rc.ch[0]);
    cloud_control_pitch(rc.rc.ch[1]);
}
