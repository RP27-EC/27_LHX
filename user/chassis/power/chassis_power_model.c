#include "chassis_power_model.h"
#include <math.h>
#include <float.h>
#include <stddef.h>

#define MODEL_SEARCH_STEPS 18U // 二分求解精度。

bool ChassisPowerModel_ConfigValid(const ChassisPowerModelConfig *config)
{
    unsigned i, j;
    if (config == NULL) { return false; }
    for (i = 0; i < CHASSIS_POWER_WHEEL_COUNT; ++i)
    {
        for (j = 0; j < CHASSIS_POWER_COEFFICIENT_COUNT; ++j)
        {
            float value = config->coefficient[i][j];
            if (!(value >= -FLT_MAX && value <= FLT_MAX)) { return false; }
        }
        // 非负二次项保证共同缩放时的功率约束为凸区间。
        if (config->coefficient[i][4] < 0.0f) { return false; }
    }
    return true;
}

float ChassisPowerModel_Predict(const ChassisPowerModelConfig *config,
                               const int16_t current[4], const int16_t rpm[4], float scale)
{
    unsigned index;
    float sum = 0.0f;
    if (config == NULL || current == NULL || rpm == NULL ||
        !(scale >= 0.0f && scale <= 1.0f)) { return FLT_MAX; }
    for (index = 0; index < CHASSIS_POWER_WHEEL_COUNT; ++index)
    {
        const float *k = config->coefficient[index];
        float i = current[index] * scale, n = rpm[index];
        float p = k[0] + k[1]*i + k[2]*n + k[3]*i*n + k[4]*i*i + k[5]*n*n;
        if (!(p >= -FLT_MAX && p <= FLT_MAX)) { return FLT_MAX; }
        if (p > 0.0f) { sum += p; }
        if (!(sum <= FLT_MAX)) { return FLT_MAX; }
    }
    return sum;
}

float ChassisPowerModel_Limit(const ChassisPowerModelConfig *config,
                             const int16_t current[4], const int16_t rpm[4], float limit_w)
{
    unsigned step;
    float lower = 0.0f, upper = 1.0f;
    float rounding_margin = 0.0f;
    if (current == NULL || rpm == NULL || !ChassisPowerModel_ConfigValid(config) ||
        !(limit_w > 0.0f && limit_w <= FLT_MAX)) { return 0.0f; }
    // 先确认零电流端可行，使后续反馈进一步缩小比例仍处于同一可行区间。
    if (ChassisPowerModel_Predict(config, current, rpm, 0.0f) > limit_w) { return 0.0f; }
    if (ChassisPowerModel_Predict(config, current, rpm, 1.0f) <= limit_w) { return 1.0f; }
    // 电流取整最多改变一个原始码，按模型斜率预留对应功率余量。
    for (step = 0; step < CHASSIS_POWER_WHEEL_COUNT; ++step)
    {
        const float *k = config->coefficient[step];
        rounding_margin += fabsf(k[1] + k[3] * rpm[step]) +
            k[4] * (2.0f * fabsf((float)current[step]) + 1.0f);
    }
    limit_w -= rounding_margin;
    // 仅转速损耗已超额度时，撤销驱动电流。
    if (ChassisPowerModel_Predict(config, current, rpm, 0.0f) > limit_w) { return 0.0f; }
    for (step = 0; step < MODEL_SEARCH_STEPS; ++step)
    {
        float middle = 0.5f * (lower + upper);
        if (ChassisPowerModel_Predict(config, current, rpm, middle) <= limit_w)
        { lower = middle; }
        else { upper = middle; }
    }
    return lower;
}

// 绑定现有状态与函数，供外部通过模块结构体访问。
const ChassisPowerModelAlgorithm chassis_power_model_algorithm =
{
    .ops = {
        .config_valid = ChassisPowerModel_ConfigValid,
        .predict = ChassisPowerModel_Predict,
        .limit = ChassisPowerModel_Limit,
    }
};
