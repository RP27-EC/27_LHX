#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H
#include "chassis_power_model.h"
typedef struct
{
    float power_w; // 超电实测底盘功率，W。
    float target_w; // 扣除余量后的功率目标，W。
    float predicted_request_w; // 限流前四轮预测总功率。
    float predicted_output_w; // 最终电流对应的预测总功率。
    float model_scale; // 四轮分配后的最小电流比例，用于观察。
    float feedback_scale; // 实测反馈修正后的功率预算比例。
    float current_scale; // 在线轮最终最小电流比例，用于观察。
    float allocation_budget_w; // 本周期分配器可用功率，W。
    float wheel_scale[4]; // 各轮实际电流与请求的比例。
    float wheel_budget_w[4]; // 各轮分到的模型功率额度。
    float target_speed_rpm[4]; // 速度或位置闭环的目标转子速度。
    bool speed_targets_valid; // 当前命令是否携带速度目标。
    bool learning_sample_aligned; // 新功率帧是否具备四轮同步反馈。
    uint16_t referee_limit_w; // 最近收到的裁判功率上限。
    bool referee_output_allowed; // 裁判底盘输出许可。
    bool referee_fresh; // 裁判机器人状态是否有效。
    bool power_feedback_valid; // 超电功率反馈是否有效。
    bool motor_feedback_valid; // 四轮反馈是否全部有效，仅用于观察。
    bool wheel_feedback_valid[4]; // 各轮反馈有效性，离线轮输出零电流。
    bool model_valid; // 模型参数是否有效。
    bool control_config_valid; // 实测闭环参数是否有效。
    bool limited; // 本周期是否限流。
    uint32_t power_sample_count; // 超电功率样本计数。
    uint32_t power_rx_ms; // 超电功率接收时间。
    uint32_t last_update_ms; // 最近控制时间。
    int16_t speed_rpm[4]; // 模型使用的四轮转子速度。
    int16_t feedback_raw[4]; // 四轮反馈力矩电流原始码。
    int16_t request_raw[4]; // 限流前电流指令。
    int16_t output_raw[4]; // 最终电流指令。
} ChassisPowerState;
extern volatile ChassisPowerState chassis_power_state;
void ChassisPower_Init(void);
float ChassisPower_Apply(int16_t currents[CHASSIS_POWER_WHEEL_COUNT]);
// 速度和位置闭环使用此入口，把本周期目标速度交给功率分配器。
float ChassisPower_ApplyWithTargets(int16_t currents[CHASSIS_POWER_WHEEL_COUNT], const float targets[4]);
uint16_t ChassisPower_GetLimit(uint32_t now_ms, bool *output_allowed);
#endif
