/**
 ******************************************************************************
 * @file    PID.h
 * @brief   通用位置式 / 增量式 PID 控制器接口
 ******************************************************************************
 */

#ifndef __PID_H
#define __PID_H

/* PID 控制器状态与参数。所有量使用同一物理单位体系。 */
typedef struct
{
    float Kp;                 /* 比例系数 */
    float Ki;                 /* 积分系数（单位：1/s） */
    float Kd;                 /* 微分系数（单位：s） */
    float IntegralLimit;      /* 积分累加值绝对限幅 */
    float OutputLimit;        /* 控制输出绝对限幅 */
    float Output;             /* 本次控制输出 */
    float SetPoint;           /* 目标值 */
    float Error;              /* 当前误差：目标值 - 测量值 */
    float LastError;          /* 上一次误差 */
    float PreviousError;      /* 上上次误差，供增量式 PID 使用 */
    float Integral;           /* 误差积分值 */
    float ControlTime;        /* 固定控制周期，单位：s */
} PID_Controller_t;

/* 保留简写类型，兼容已有 PID_t 写法。 */
typedef PID_Controller_t PID_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit, float control_time);
float PID_Calc(PID_Controller_t *pid, float setpoint, float current_value);
float PID_Calc_Incremental(PID_Controller_t *pid, float setpoint, float current_value);
void PID_Reset(PID_Controller_t *pid);

#endif /* __PID_H */
