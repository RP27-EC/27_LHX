#include "chassis_power_control.h"
#include <float.h>
#include <stddef.h>

static float clamp(float value, float lower, float upper)
{ return value < lower ? lower : (value > upper ? upper : value); }
static bool nonnegative(float value)
{ return value >= 0.0f && value <= FLT_MAX; }

bool ChassisPowerControl_ConfigValid(const ChassisPowerControlConfig *config)
{
    return config != NULL && nonnegative(config->offline_limit_w) && config->offline_limit_w <= 65535.0f &&
        nonnegative(config->reserve_w) && config->reserve_w <= 65535.0f &&
        nonnegative(config->deadband_w) && config->deadband_w <= 65535.0f &&
        nonnegative(config->kp) && config->kp <= 1000.0f &&
        nonnegative(config->ki_per_s) && config->ki_per_s <= 1000.0f &&
        nonnegative(config->recovery_per_s) && config->recovery_per_s <= 1000.0f &&
        nonnegative(config->initial_scale) && config->initial_scale <= 1.0f &&
        config->offline_current_limit >= 0 && config->offline_current_limit <= 16384;
}

void ChassisPowerControl_Init(ChassisPowerController *controller, float initial_scale)
{
    if (controller == NULL) { return; }
    if (!nonnegative(initial_scale)) { initial_scale = 0.0f; }
    controller->integral = clamp(initial_scale, 0.0f, 1.0f);
    controller->scale = controller->integral;
}

float ChassisPowerControl_Update(ChassisPowerController *controller,
                                const ChassisPowerControlConfig *config,
                                float power_w, float target_w, float dt_s)
{
    float error, candidate, output, upper;
    if (controller == NULL) { return 0.0f; }
    if (!ChassisPowerControl_ConfigValid(config) || !nonnegative(power_w) ||
        !nonnegative(target_w) || !nonnegative(dt_s))
    { controller->scale = 0.0f; return 0.0f; }
    if (target_w == 0.0f)
    { controller->scale = 0.0f; controller->integral = 0.0f; return 0.0f; }
    // 用相对误差，使不同功率档位使用同一组控制增益。
    error = target_w - power_w;
    if (error <= config->deadband_w && error >= -config->deadband_w) { error = 0.0f; }
    error /= target_w;
    if (!(error >= -FLT_MAX && error <= FLT_MAX))
    { controller->scale = 0.0f; return 0.0f; }
    dt_s = clamp(dt_s, 0.0f, 0.1f);
    candidate = clamp(controller->integral + config->ki_per_s * error * dt_s, 0.0f, 1.0f);
    output = candidate + config->kp * error;
    // 输出饱和且误差继续加剧时，不再累积同方向积分。
    if ((output > 1.0f && error > 0.0f) || (output < 0.0f && error < 0.0f))
    { output = controller->integral + config->kp * error; }
    else { controller->integral = candidate; }
    output = clamp(output, 0.0f, 1.0f);
    // 超限时立即允许收紧，恢复时限制上升速度。
    upper = clamp(controller->scale + config->recovery_per_s * dt_s, 0.0f, 1.0f);
    if (output > upper) { output = upper; }
    controller->scale = output;
    return output;
}
