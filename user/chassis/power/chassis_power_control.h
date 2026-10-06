#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include <stdbool.h>

typedef struct
{
    bool enabled; // 自适应模型与实测反馈限制总开关。
    float offline_limit_w; // 裁判未接或过期时的备用上限。
    float reserve_w; // 从裁判上限扣除的功率余量。
    float deadband_w; // 目标附近的功率误差死区。
    float kp; // 归一化功率误差到功率预算比例的比例增益。
    float ki_per_s; // 功率预算比例积分的每秒增益。
    float recovery_per_s; // 功率预算比例每秒允许增加的幅度。
    int offline_current_limit; // 超电未收到或过期时的单轮电流原值上限。
} ChassisPowerControlConfig;

typedef struct
{
    float integral; // 比例控制的积分状态。
    float scale; // 当前模型可用功率预算比例。
} ChassisPowerController;

void ChassisPowerControl_Init(ChassisPowerController *controller, float initial_scale);
bool ChassisPowerControl_ConfigValid(const ChassisPowerControlConfig *config);
// 每个新功率样本调用一次；目标改变时可用 dt=0 立即更新比例项。
float ChassisPowerControl_Update(ChassisPowerController *controller,
                                const ChassisPowerControlConfig *config,
                                float power_w, float target_w, float dt_s);

// 共用 PI 入口，每个功率控制器保存独立状态。
typedef struct
{
    void (*init)(ChassisPowerController *controller, float initial_scale); // 建立积分与输出初值。
    bool (*config_valid)(const ChassisPowerControlConfig *config); // 检查反馈控制参数。
    float (*update)(ChassisPowerController *controller,
                    const ChassisPowerControlConfig *config,
                    float power_w, float target_w, float dt_s); // 更新统一功率预算比例。
} ChassisPowerControlOps;

typedef struct
{
    ChassisPowerControlOps ops; // 实测功率反馈操作表。
} ChassisPowerControlAlgorithm;

extern const ChassisPowerControlAlgorithm chassis_power_pi_algorithm; // 功率 PI 算法入口。
#endif
