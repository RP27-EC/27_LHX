#include "chassis_power_estimate.h"
#include <float.h>
#include <string.h>

static bool positive(float value)
{ return value > 0.0f && value <= FLT_MAX; }

static float absolute(float value)
{ return value < 0.0f ? -value : value; }

void ChassisPowerEstimate_Calculate(const ChassisPowerConfig *config,
                                    const ChassisPowerInput *input,
                                    const int16_t currents[CHASSIS_POWER_WHEEL_COUNT],
                                    ChassisPowerResult *result)
{
    uint32_t i;
    float output_sum = 0.0f, feedback_sum = 0.0f, power;
    if (config == NULL || input == NULL || currents == NULL || result == NULL)
    { return; }
    memset(result, 0, sizeof(*result));
    result->config_valid = positive(config->estimate_gain) &&
        positive(config->command_a_per_raw) && positive(config->feedback_a_per_raw);
    if (!result->config_valid) { return; }
    for (i = 0U; i < CHASSIS_POWER_WHEEL_COUNT; i++)
    {
        output_sum += absolute((float)currents[i]);
        if (input->feedback_valid)
        {
            result->feedback_current_a[i] = (float)input->feedback_raw[i] * config->feedback_a_per_raw;
            feedback_sum += absolute((float)input->feedback_raw[i]);
        }
    }
    result->output_current_abs_sum_a = output_sum * config->command_a_per_raw;
    result->feedback_current_abs_sum_a = feedback_sum * config->feedback_a_per_raw;
    if (!input->voltage_valid || !(input->voltage_v >= 0.0f && input->voltage_v <= FLT_MAX))
    { return; }
    // 反馈与指令分别显示，方向相反的电流也不会抵消。
    power = input->voltage_v * result->output_current_abs_sum_a * config->estimate_gain;
    result->output_estimate_valid = power >= 0.0f && power <= FLT_MAX;
    if (result->output_estimate_valid) { result->output_estimate_w = power; }
    power = input->voltage_v * result->feedback_current_abs_sum_a * config->estimate_gain;
    result->feedback_estimate_valid = input->feedback_valid && power >= 0.0f && power <= FLT_MAX;
    if (result->feedback_estimate_valid) { result->feedback_estimate_w = power; }
}
