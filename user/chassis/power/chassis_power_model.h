#ifndef CHASSIS_POWER_MODEL_H
#define CHASSIS_POWER_MODEL_H
#include <stdbool.h>
#include <stdint.h>
#include "rls2.h"

#define CHASSIS_POWER_WHEEL_COUNT 4U // 底盘驱动轮数。

typedef struct
{
    float k1; // 输出轴角速度损耗系数，W/(rad/s)。
    float k2; // 输出轴力矩平方损耗系数，W/(N*m)^2。
    float static_loss_w; // 四个电调的固定损耗总和，W。
} ChassisPowerLoss;

typedef struct
{
    struct {
        float torque_nm_per_raw; // C620 原始电流码到减速器输出轴力矩。
        float reduction_ratio; // 转子 rpm 转输出轴速度所用减速比。
    } motor;
    ChassisPowerLoss initial; // 上电模型，在线学习从此处开始。
    struct {
        bool enabled; // 在线修正速度损耗和力矩损耗。
        float forgetting; // RLS 遗忘系数，越接近一越重视历史。
        float initial_covariance; // 初始参数不确定度。
        float minimum_power_w; // 低于此实测功率时停止学习。
        float maximum_innovation_w; // 拒绝过大的功率残差。
        uint32_t alignment_window_ms; // 功率帧与各轮反馈的最大时间差。
        float k1_min, k1_max; // 速度损耗系数的允许范围。
        float k2_min, k2_max; // 力矩损耗系数的允许范围。
    } learning;
    struct {
        float error_blend_low_rad_s; // 总速度误差低于此值按请求功率分配。
        float error_blend_high_rad_s; // 总速度误差高于此值按速度误差分配。
    } allocation;
} ChassisPowerModelConfig;

typedef struct
{
    ChassisPowerLoss loss; // 当前参与预测的损耗参数。
    Rls2 estimator; // 两项损耗的在线辨识历史。
    uint32_t rejected_samples; // 跳过或拒绝的学习样本数。
    float feedback_prediction_w; // 实测电流和速度对应的带符号总功率。
    float error_confidence; // 分配对速度误差的依赖程度。
    bool sample_used; // 本周期是否接受了新功率样本。
} ChassisPowerModelState;

bool ChassisPowerModel_ConfigValid(const ChassisPowerModelConfig *config);
void ChassisPowerModel_Init(ChassisPowerModelState *state, const ChassisPowerModelConfig *config);
float ChassisPowerModel_WheelPower(const ChassisPowerModelConfig *config, const ChassisPowerLoss *loss,
                                  float current_raw, int16_t rpm);
// 只计在线轮的正功率，用于输出限额；学习使用带符号总功率。
float ChassisPowerModel_Predict(const ChassisPowerModelConfig *config, const ChassisPowerLoss *loss,
                               const int16_t current[4], const int16_t rpm[4], const bool online[4]);
bool ChassisPowerModel_Learn(ChassisPowerModelState *state, const ChassisPowerModelConfig *config,
                            const int16_t feedback[4], const int16_t rpm[4], float measured_w);
// targets 为转子 rpm；直接电流模式传 NULL，按各轮请求功率分配。
void ChassisPowerModel_Allocate(ChassisPowerModelState *state, const ChassisPowerModelConfig *config,
                               int16_t current[4], const int16_t rpm[4], const bool online[4],
                               const float targets[4], float budget_w, float wheel_budget[4]);

typedef struct
{
    bool (*config_valid)(const ChassisPowerModelConfig *config);
    void (*init)(ChassisPowerModelState *state, const ChassisPowerModelConfig *config);
    float (*predict)(const ChassisPowerModelConfig *config, const ChassisPowerLoss *loss,
                     const int16_t current[4], const int16_t rpm[4], const bool online[4]);
    bool (*learn)(ChassisPowerModelState *state, const ChassisPowerModelConfig *config,
                  const int16_t feedback[4], const int16_t rpm[4], float measured_w);
    void (*allocate)(ChassisPowerModelState *state, const ChassisPowerModelConfig *config,
                     int16_t current[4], const int16_t rpm[4], const bool online[4],
                     const float targets[4], float budget_w, float wheel_budget[4]);
} ChassisPowerModelOps;
typedef struct { ChassisPowerModelOps ops; } ChassisPowerModelAlgorithm;
extern const ChassisPowerModelAlgorithm chassis_power_model_algorithm; // 自适应模型共用入口。
extern ChassisPowerModelState chassis_power_model_state; // 下板功率模型运行数据。
#endif
