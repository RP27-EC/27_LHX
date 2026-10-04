#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include <stdbool.h>

typedef struct
{
    bool enabled; // 模型与实测功率限制总开关。
    float offline_limit_w; // 裁判未接或过期时的备用上限。
    float reserve_w; // 从裁判上限扣除的功率余量。
    float deadband_w; // 目标附近的功率误差死区。
    float kp; // 归一化功率误差到电流比例的比例增益。
    float ki_per_s; // 电流比例积分的每秒增益。
    float recovery_per_s; // 电流比例每秒允许增加的幅度。
    float initial_scale; // 初始化时的电流比例。
    int offline_current_limit; // 超电未收到或过期时的单轮电流原值上限。
} ChassisPowerControlConfig;

typedef struct
{
    float integral; // 比例控制的积分状态。
    float scale; // 当前四轮统一电流比例。
} ChassisPowerController;

void ChassisPowerControl_Init(ChassisPowerController *controller, float initial_scale);
bool ChassisPowerControl_ConfigValid(const ChassisPowerControlConfig *config);
// 每个新功率样本调用一次；目标改变时可用 dt=0 立即更新比例项。
float ChassisPowerControl_Update(ChassisPowerController *controller,
                                const ChassisPowerControlConfig *config,
                                float power_w, float target_w, float dt_s);
#endif
