// 位置式与增量式 PID 不能共用同一个实例。

#include "PID.h"

// 双向限幅。
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

// 浮点绝对值。
static float PID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

// 初始化参数并清空状态；control_time 单位为秒。
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
    pid->ConfigKp = kp;
    pid->ConfigKi = ki;
    pid->ConfigKd = kd;
    pid->ConfigIntegralLimit = integral_limit;
    pid->ConfigOutputLimit = output_limit;
    pid->ConfigControlTime = control_time;

    PID_Reset(pid);
}

void PID_UpdateParameters(PID_Controller_t *pid, float kp, float ki, float kd,
                          float integral_limit, float output_limit,
                          float control_time)
{
    if (pid == 0) { return; }
    if (pid->ConfigKp == kp && pid->ConfigKi == ki &&
        pid->ConfigKd == kd &&
        pid->ConfigIntegralLimit == integral_limit &&
        pid->ConfigOutputLimit == output_limit &&
        pid->ConfigControlTime == control_time)
    { return; }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->IntegralLimit = PID_Abs(integral_limit);
    pid->OutputLimit = PID_Abs(output_limit);
    if (control_time > 0.0f) { pid->ControlTime = control_time; }
    pid->Integral = PID_Clamp(pid->Integral, pid->IntegralLimit);
    pid->Output = PID_Clamp(pid->Output, pid->OutputLimit);
    pid->ConfigKp = kp;
    pid->ConfigKi = ki;
    pid->ConfigKd = kd;
    pid->ConfigIntegralLimit = integral_limit;
    pid->ConfigOutputLimit = output_limit;
    pid->ConfigControlTime = control_time;
}

// 位置式 PID，带积分和输出限幅。
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

    // 先计算候选积分值；输出已饱和且误差仍会加剧饱和时，不再积分。
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

// 增量式 PID，返回累加后的限幅输出。
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

// 清空控制状态，保留参数。
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

// 绑定现有状态与函数，供外部通过模块结构体访问。
const PidAlgorithm pid_algorithm =
{
    .ops = {
        .init = PID_Init,
        .update_parameters = PID_UpdateParameters,
        .calc = PID_Calc,
        .calc_incremental = PID_Calc_Incremental,
        .reset = PID_Reset,
    }
};
