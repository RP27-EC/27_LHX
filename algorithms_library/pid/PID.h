// 位置式与增量式 PID 接口。

#ifndef __PID_H
#define __PID_H

// PID 控制器状态与参数。所有量使用同一物理单位体系。
typedef struct
{
    float Kp; // 比例系数
    float Ki; // 积分系数（单位：1/s）
    float Kd; // 微分系数（单位：s）
    float IntegralLimit; // 积分累加值绝对限幅
    float OutputLimit; // 控制输出绝对限幅
    float Output; // 本次控制输出
    float SetPoint; // 目标值
    float Error; // 当前误差：目标值 - 测量值
    float LastError; // 上一次误差
    float PreviousError; // 上上次误差，供增量式 PID 使用
    float Integral; // 误差积分值
    float ControlTime; // 固定控制周期，单位：s
    float ConfigKp; // 最近一次同步的配置值。
    float ConfigKi;
    float ConfigKd;
    float ConfigIntegralLimit;
    float ConfigOutputLimit;
    float ConfigControlTime;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit, float control_time);
// 仅配置值变化时同步；直接修改 PID 成员不会被周期覆盖。
void PID_UpdateParameters(PID_Controller_t *pid, float kp, float ki, float kd,
                          float integral_limit, float output_limit,
                          float control_time);
float PID_Calc(PID_Controller_t *pid, float setpoint, float current_value);
float PID_Calc_Incremental(PID_Controller_t *pid, float setpoint, float current_value);
void PID_Reset(PID_Controller_t *pid);


// 共用算法入口；各控制器实例保存自己的参数和运行状态。
typedef struct
{
    void (*init)(
        PID_Controller_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit,
        float control_time); // 初始化模块。
    void (*update_parameters)(
        PID_Controller_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit,
        float control_time); // 同步变化后的 PID 参数。
    float (*calc)(PID_Controller_t *pid, float setpoint, float current_value); // 计算位置式 PID 输出。
    float (*calc_incremental)(PID_Controller_t *pid, float setpoint, float current_value); // 计算增量式 PID 输出。
    void (*reset)(PID_Controller_t *pid); // 清空 PID 积分和误差状态。
} PidAlgorithmOps;

typedef struct
{
    PidAlgorithmOps ops; // 固定操作接口，随程序保存。
} PidAlgorithm;

extern const PidAlgorithm pid_algorithm; // 模块统一访问入口。

#endif // __PID_H
