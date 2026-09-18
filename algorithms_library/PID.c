/**
 ******************************************************************************
 * @file    PID.c
 * @brief   位置式 / 增量式 PID 控制器实现
 * @note    PID_Calc 与 PID_Calc_Incremental 不能在同一个控制器实例中混用。
 ******************************************************************************
 */

#include "PID.h"

/**
  * @brief  将数值限制在指定正负范围内。
  */
static float PID_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/**
  * @brief  获取浮点数绝对值，避免引入额外数学库依赖。
  */
static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
  * @brief  初始化 PID 参数，并清空历史控制状态。
  * @param  control_time: 固定调用周期，单位为秒，例如 0.01f 表示 10 ms。
  */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit, float control_time)
{
    if (pid == 0)
    {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->IntegralLimit = PID_Abs(integral_limit);
    pid->OutputLimit = PID_Abs(output_limit);
    pid->ControlTime = (control_time > 0.0f) ? control_time : 0.001f;

    PID_Reset(pid);
}

/**
  * @brief  位置式 PID 计算，带积分限幅和抗积分饱和。
  * @retval 限幅后的绝对控制输出。
  */
float PID_Calc(PID_Controller_t *pid, float setpoint, float current_value)
{
    float candidate_integral;
    float derivative;
    float raw_output;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->SetPoint = setpoint;
    pid->Error = setpoint - current_value;
    derivative = (pid->Error - pid->LastError) / pid->ControlTime;

    /* 先计算候选积分值；输出已饱和且误差仍会加剧饱和时，不再积分。 */
    candidate_integral = PID_Clamp(pid->Integral + pid->Error * pid->ControlTime,
                                   pid->IntegralLimit);
    raw_output = pid->Kp * pid->Error
               + pid->Ki * candidate_integral
               + pid->Kd * derivative;

    if (((raw_output > pid->OutputLimit) && (pid->Error > 0.0f)) ||
        ((raw_output < -pid->OutputLimit) && (pid->Error < 0.0f)))
    {
        raw_output = pid->Kp * pid->Error
                   + pid->Ki * pid->Integral
                   + pid->Kd * derivative;
    }
    else
    {
        pid->Integral = candidate_integral;
    }

    pid->Output = PID_Clamp(raw_output, pid->OutputLimit);
    pid->PreviousError = pid->LastError;
    pid->LastError = pid->Error;

    return pid->Output;
}

/**
  * @brief  增量式 PID 计算。
  * @note   返回累加并限幅后的绝对输出，而不是单独的 Δu，便于直接驱动电机。
  */
float PID_Calc_Incremental(PID_Controller_t *pid, float setpoint, float current_value)
{
    float delta_output;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->SetPoint = setpoint;
    pid->Error = setpoint - current_value;

    delta_output = pid->Kp * (pid->Error - pid->LastError)
                 + pid->Ki * pid->Error * pid->ControlTime
                 + pid->Kd * (pid->Error - 2.0f * pid->LastError + pid->PreviousError)
                   / pid->ControlTime;

    pid->Output = PID_Clamp(pid->Output + delta_output, pid->OutputLimit);
    pid->PreviousError = pid->LastError;
    pid->LastError = pid->Error;

    return pid->Output;
}

/**
  * @brief  清空 PID 的误差与输出历史，保留已经设置好的参数。
  */
void PID_Reset(PID_Controller_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->Output = 0.0f;
    pid->SetPoint = 0.0f;
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->PreviousError = 0.0f;
    pid->Integral = 0.0f;
}
