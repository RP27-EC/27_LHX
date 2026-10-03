#ifndef CHASSIS_POWER_ESTIMATE_H
#define CHASSIS_POWER_ESTIMATE_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_POWER_WHEEL_COUNT 4U // 参与功率观测的底盘轮数。

typedef struct
{
    float estimate_gain; // 乘积估算的修正倍率，仅影响显示值。
    float command_a_per_raw; // 指令电流原值到 A 的换算比例。
    float feedback_a_per_raw; // 反馈电流原值到 A 的假定比例，需核对。
} ChassisPowerConfig;

typedef struct
{
    float voltage_v; // 超电返回的实际电容电压，不加下限或备用值。
    bool voltage_valid; // 电压数据是否新鲜且有效。
    bool feedback_valid; // 四轮反馈是否全部新鲜。
    int16_t feedback_raw[CHASSIS_POWER_WHEEL_COUNT]; // 四轮反馈力矩电流原值。
} ChassisPowerInput;

typedef struct
{
    float feedback_current_a[CHASSIS_POWER_WHEEL_COUNT]; // 四轮有符号反馈电流换算值。
    float feedback_current_abs_sum_a; // 四轮反馈电流绝对值之和。
    float output_current_abs_sum_a; // 待发指令电流绝对值之和。
    float feedback_estimate_w; // 电容电压乘反馈电流和的估算。
    float output_estimate_w; // 电容电压乘指令电流和的估算。
    bool feedback_estimate_valid; // 反馈功率估算是否有效。
    bool output_estimate_valid; // 指令功率估算是否有效。
    bool config_valid; // 换算比例与修正倍率是否有效。
} ChassisPowerResult;

// 纯观测计算，输入电流为只读，不限制输出。
void ChassisPowerEstimate_Calculate(const ChassisPowerConfig *config,
                                    const ChassisPowerInput *input,
                                    const int16_t currents[CHASSIS_POWER_WHEEL_COUNT],
                                    ChassisPowerResult *result);

#endif
