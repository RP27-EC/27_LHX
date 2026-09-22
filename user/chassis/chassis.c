#include "chassis.h"
#include "communication.h"
#include "imu.h"
#include <math.h>

#define CHASSIS_DEG_TO_RAD 0.01745329251994329577f

static float chassis_follow_cycle_rpm;
static uint32_t chassis_follow_last_rate_tx_ms;
static float chassis_spin_cycle_rpm;


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

void Chassis_SpinUpdate(float gimbal_front, float gimbal_left)
{
    const float target_rpm = CHASSIS_SPIN_ROTATE_RPM *
                             CHASSIS_SPIN_ROTATE_SIGN;
    float step = target_rpm - chassis_spin_cycle_rpm;
    float yaw_angle_deg;
    float yaw_rad;
    float cosine;
    float sine;
    float chassis_front = 0.0f;
    float chassis_left = 0.0f;

    if (step > CHASSIS_SPIN_SLEW_RPM_PER_TICK)
    { step = CHASSIS_SPIN_SLEW_RPM_PER_TICK; }
    else if (step < -CHASSIS_SPIN_SLEW_RPM_PER_TICK)
    { step = -CHASSIS_SPIN_SLEW_RPM_PER_TICK; }
    chassis_spin_cycle_rpm += step;

    if (Communication_GetYawAngle(&yaw_angle_deg))
    {
        /* 遥控平移量定义在云台坐标系。使用云台相对底盘的机械Yaw角
         * 旋转到底盘坐标系，使“向前”始终等于云台当前指向。 */
        yaw_rad = yaw_angle_deg * CHASSIS_SPIN_YAW_ANGLE_SIGN *
                  CHASSIS_DEG_TO_RAD;
        cosine = cosf(yaw_rad);
        sine = sinf(yaw_rad);
        chassis_front = gimbal_front * cosine - gimbal_left * sine;
        chassis_left = gimbal_front * sine + gimbal_left * cosine;
    }

    /* C1角度暂时无效时平移保持为零，但小陀螺自转继续运行。 */
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

    /* 连续软死区：10 度内不追，越过边界时从零速平滑起步。 */
    if (angle_deg > CHASSIS_FOLLOW_DEADBAND_DEG)
    { error_deg = angle_deg - CHASSIS_FOLLOW_DEADBAND_DEG; }
    else if (angle_deg < -CHASSIS_FOLLOW_DEADBAND_DEG)
    { error_deg = angle_deg + CHASSIS_FOLLOW_DEADBAND_DEG; }
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

