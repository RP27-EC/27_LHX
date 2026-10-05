#ifndef CHASSIS_POWER_MODEL_H
#define CHASSIS_POWER_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_POWER_WHEEL_COUNT 4U // 底盘驱动轮数。
#define CHASSIS_POWER_COEFFICIENT_COUNT 6U // 二次电流转速模型的项数。

typedef struct
{
    float coefficient[CHASSIS_POWER_WHEEL_COUNT][CHASSIS_POWER_COEFFICIENT_COUNT]; // 输入为电流原始码、转子 rpm，输出 W。
} ChassisPowerModelConfig;

bool ChassisPowerModel_ConfigValid(const ChassisPowerModelConfig *config);
// 负功率按回馈处理，累计值只计各轮正功率。
float ChassisPowerModel_Predict(const ChassisPowerModelConfig *config,
                               const int16_t current[4], const int16_t rpm[4], float scale);
// 保持四轮电流比例，求满足功率额度的最大缩放值。
float ChassisPowerModel_Limit(const ChassisPowerModelConfig *config,
                             const int16_t current[4], const int16_t rpm[4], float limit_w);

// 共用功率预测入口，模型参数由调用者传入。
typedef struct
{
    bool (*config_valid)(const ChassisPowerModelConfig *config); // 检查模型系数。
    float (*predict)(const ChassisPowerModelConfig *config,
                     const int16_t current[4], const int16_t rpm[4], float scale); // 预测四轮正功率。
    float (*limit)(const ChassisPowerModelConfig *config,
                   const int16_t current[4], const int16_t rpm[4], float limit_w); // 求允许的电流比例。
} ChassisPowerModelOps;

typedef struct
{
    ChassisPowerModelOps ops; // 功率模型操作表。
} ChassisPowerModelAlgorithm;

extern const ChassisPowerModelAlgorithm chassis_power_model_algorithm; // 功率预测算法入口。

#endif
