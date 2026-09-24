#include "chassis.h"
#include "communication.h"
#include "imu.h"
#include <math.h>

/* 角度制转弧度制：1°=π/180 rad，供云台相对车头角的坐标旋转使用。 */
#define CHASSIS_DEG_TO_RAD 0.01745329251994329577f

static float chassis_follow_cycle_rpm; /* 跟随模式当前旋转分量，带斜坡变化。 */
static uint32_t chassis_follow_last_rate_tx_ms; /* 最近发送底盘实测角速度的时间。 */
static float chassis_spin_cycle_rpm; /* 小陀螺模式当前自旋分量，带斜坡变化。 */
volatile bool chassis_front_reversed; /* 当前更接近云台指向的车头：false=物理前，true=物理后。 */
volatile bool chassis_turnaround_pending; /* 本地拨轮已触发，等待上板完成 Yaw 调头。 */
static bool chassis_turnaround_seen_active; /* 已收到上板正在调头的 C1 标志。 */
static bool chassis_turnaround_target_reversed; /* 本次主动调头要切换到的车头方向。 */
static uint32_t chassis_turnaround_last_request_count;


static float Chassis_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Chassis_Max4(float value_0,float value_1,float value_2,float value_3)
{
    float maximum = value_0;

    if (value_1 > maximum) { maximum = value_1; }
    if (value_2 > maximum) { maximum = value_2; }
    if (value_3 > maximum) { maximum = value_3; }

    return maximum;
}

static void Chassis_SelectNearestFront(float angle_deg)
{
    float magnitude = Chassis_Abs(angle_deg);
    if (magnitude >= CHASSIS_FRONT_SWITCH_DEG)
    { chassis_front_reversed = true; }
    else { chassis_front_reversed = false; }
}

/* 遥控平移量定义在云台坐标系；用机械 Yaw 角旋转到底盘坐标系。 */
static void Chassis_GimbalToBase(float angle_deg, float *front, float *left)
{
    float yaw_rad = angle_deg * CHASSIS_SPIN_YAW_ANGLE_SIGN *
                    CHASSIS_DEG_TO_RAD;
    float cosine = cosf(yaw_rad);
    float sine = sinf(yaw_rad);
    float gimbal_front = *front;
    float gimbal_left = *left;
    *front = gimbal_front * cosine - gimbal_left * sine;
    *left = gimbal_front * sine + gimbal_left * cosine;
}

void Chassis_TurnaroundReset(uint32_t request_count)
{
    chassis_turnaround_pending = false;
    chassis_turnaround_seen_active = false;
    chassis_turnaround_last_request_count = request_count;
}

bool Chassis_TurnaroundUpdate(uint32_t request_count)
{
    float angle_deg = 0.0f;
    bool turning = false;
    bool angle_valid = Communication_GetYawState(&angle_deg, &turning);
    float target_error;

    if (request_count != chassis_turnaround_last_request_count)
    {
        chassis_turnaround_last_request_count = request_count;
        if (!chassis_turnaround_pending && !(angle_valid && turning))
        {
            if (angle_valid) { Chassis_SelectNearestFront(angle_deg); }
            chassis_turnaround_target_reversed = !chassis_front_reversed;
            chassis_turnaround_pending = true;
            chassis_turnaround_seen_active = false;
        }
    }

    if (chassis_turnaround_pending && angle_valid)
    {
        if (turning) { chassis_turnaround_seen_active = true; }
        target_error = chassis_turnaround_target_reversed ?
            180.0f - Chassis_Abs(angle_deg) : Chassis_Abs(angle_deg);
        if (chassis_turnaround_seen_active && !turning &&
            target_error <= CHASSIS_TURN_DONE_TOLERANCE_DEG)
        {
            chassis_turnaround_pending = false;
            chassis_turnaround_seen_active = false;
        }
    }
    if (!chassis_turnaround_pending && angle_valid)
    { Chassis_SelectNearestFront(angle_deg); }
    /* 即使下板漏掉本地边沿，只要上板报告正在调头也立即停轮。 */
    return chassis_turnaround_pending || (angle_valid && turning);
}

void Chassis_MechanicalUpdate(float front, float left, float cycle)
{
    float angle_deg;
    if (Communication_GetYawAngle(&angle_deg))
    {
        Chassis_SelectNearestFront(angle_deg);
        Chassis_GimbalToBase(angle_deg, &front, &left);
    }
    else if (chassis_front_reversed)
    {
        /* C1 暂时失效时沿用上次确认的正方向，保留手动底盘控制。 */
        front = -front;
        left = -left;
    }
    Chassis_MecanumInverse(front, left, cycle);
}

void Chassis_MecanumInverse(float front,float left,float cycle)
{
    static float motor[4];
    uint32_t i;

    motor[0] =  front - left - cycle;
    motor[1] =  front + left - cycle;
    motor[2] = -front - left - cycle;
    motor[3] = -front + left - cycle;

    float max_abs = Chassis_Max4(Chassis_Abs(motor[0]),
                                 Chassis_Abs(motor[1]),
                                 Chassis_Abs(motor[2]),
                                 Chassis_Abs(motor[3]));

    if (max_abs > CHASSIS_MAX_MOTOR_RPM)
    {
        float scale = CHASSIS_MAX_MOTOR_RPM / max_abs;

        for (i = 0; i < 4; i++)
        {
            motor[i] *= scale;
        }
    }

    (void)Motor3508_control(1U,
                            (int16_t)motor[0],
                            (int16_t)motor[1],
                            (int16_t)motor[2],
                            (int16_t)motor[3]);
}

void Chassis_FollowReset(void)
{
    chassis_follow_cycle_rpm = 0.0f;
    chassis_follow_last_rate_tx_ms = 0U;
}

void Chassis_SpinReset(void)
{
    chassis_spin_cycle_rpm = 0.0f;
}

void Chassis_SpinUpdate(float gimbal_front, float gimbal_left,
                        bool spin_enabled)
{
    const float target_rpm = spin_enabled ?
        CHASSIS_SPIN_ROTATE_RPM * CHASSIS_SPIN_ROTATE_SIGN : 0.0f;
    float step = target_rpm - chassis_spin_cycle_rpm;
    float yaw_angle_deg;
    float chassis_front = 0.0f;
    float chassis_left = 0.0f;

    if (!spin_enabled)
    {
        /* 右拨杆离开上档时立即撤销自旋指令。 */
        chassis_spin_cycle_rpm = 0.0f;
    }
    else
    {
        if (step > CHASSIS_SPIN_SLEW_RPM_PER_TICK)
        { step = CHASSIS_SPIN_SLEW_RPM_PER_TICK; }
        else if (step < -CHASSIS_SPIN_SLEW_RPM_PER_TICK)
        { step = -CHASSIS_SPIN_SLEW_RPM_PER_TICK; }
        chassis_spin_cycle_rpm += step;
    }

    if (Communication_GetYawAngle(&yaw_angle_deg))
    {
        /* 遥控平移量定义在云台坐标系。使用云台相对底盘的机械Yaw角
         * 旋转到底盘坐标系，使“向前”始终等于云台当前指向。 */
        Chassis_SelectNearestFront(yaw_angle_deg);
        chassis_front = gimbal_front;
        chassis_left = gimbal_left;
        Chassis_GimbalToBase(yaw_angle_deg, &chassis_front, &chassis_left);
    }

    /* C1角度暂时无效时平移为零；自旋仍受右拨杆上档控制。 */
    Chassis_MecanumInverse(chassis_front, chassis_left,
                           chassis_spin_cycle_rpm);
}

void Chassis_FollowUpdate(float front, float left, float yaw_input)
{
    float angle_deg, error_deg, feedforward_rpm;
    float target_rpm, step, rate_deg_s;
    uint32_t now_ms;

    if (!Communication_GetYawAngle(&angle_deg) ||
        !ChassisImu_GetYawRate(&rate_deg_s))
    {
        Chassis_FollowReset();
        (void)Motor3508_Stop();
        return;
    }

    Chassis_SelectNearestFront(angle_deg);
    /* 两个相反的车头分别是 0° 与 ±180°，选离云台最近的一侧。 */
    if (chassis_front_reversed)
    { error_deg = angle_deg - (angle_deg >= 0.0f ? 180.0f : -180.0f); }
    else { error_deg = angle_deg; }
    Chassis_GimbalToBase(angle_deg, &front, &left);

    /* 连续软死区内不追，越过边界时从零速平滑起步。 */
    if (error_deg > CHASSIS_FOLLOW_DEADBAND_DEG)
    { error_deg -= CHASSIS_FOLLOW_DEADBAND_DEG; }
    else if (error_deg < -CHASSIS_FOLLOW_DEADBAND_DEG)
    { error_deg += CHASSIS_FOLLOW_DEADBAND_DEG; }
    else { error_deg = 0.0f; }

    /* 使用与上板Yaw相同的遥控输入作为前馈。底盘无需等待云台偏出
     * 机械角死区才开始转动；松杆后前馈归零，仍由角度闭环归中。 */
    if (yaw_input > CHASSIS_FOLLOW_RC_DEADBAND)
    {
        feedforward_rpm =
            (yaw_input - CHASSIS_FOLLOW_RC_DEADBAND) *
            CHASSIS_FOLLOW_FF_RPM_PER_RC;
    }
    else if (yaw_input < -CHASSIS_FOLLOW_RC_DEADBAND)
    {
        feedforward_rpm =
            (yaw_input + CHASSIS_FOLLOW_RC_DEADBAND) *
            CHASSIS_FOLLOW_FF_RPM_PER_RC;
    }
    else
    {
        feedforward_rpm = 0.0f;
    }

    target_rpm = (error_deg * CHASSIS_FOLLOW_KP_RPM_PER_DEG +
                  feedforward_rpm) * CHASSIS_FOLLOW_ROTATE_SIGN;
    if (target_rpm > CHASSIS_FOLLOW_MAX_ROTATE_RPM)
    { target_rpm = CHASSIS_FOLLOW_MAX_ROTATE_RPM; }
    else if (target_rpm < -CHASSIS_FOLLOW_MAX_ROTATE_RPM)
    { target_rpm = -CHASSIS_FOLLOW_MAX_ROTATE_RPM; }
    step = target_rpm - chassis_follow_cycle_rpm;
    if (step > CHASSIS_FOLLOW_SLEW_RPM_PER_TICK)
    { step = CHASSIS_FOLLOW_SLEW_RPM_PER_TICK; }
    else if (step < -CHASSIS_FOLLOW_SLEW_RPM_PER_TICK)
    { step = -CHASSIS_FOLLOW_SLEW_RPM_PER_TICK; }
    chassis_follow_cycle_rpm += step;

    Chassis_MecanumInverse(front, left, chassis_follow_cycle_rpm);
    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - chassis_follow_last_rate_tx_ms) >=
            CHASSIS_FOLLOW_RATE_TX_PERIOD_MS &&
        Communication_SendChassisYawRate(rate_deg_s) == HAL_OK)
    {
        chassis_follow_last_rate_tx_ms = now_ms;
    }
}

