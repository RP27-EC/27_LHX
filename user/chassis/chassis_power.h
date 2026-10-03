#ifndef CHASSIS_POWER_H
#define CHASSIS_POWER_H

#include "chassis_power_estimate.h"

typedef struct
{
    ChassisPowerResult estimate; // 电压与力矩电流的乘积估算，不参与控制。
    float capacitor_voltage_v; // 超电返回的电容端电压。
    uint16_t referee_limit_w; // 最后收到的裁判上限，仅供对照。
    bool referee_output_allowed; // 最后收到的裁判输出许可，仅供观察。
    bool referee_fresh; // 裁判机器人状态是否新鲜。
    bool capacitor_fresh; // 超电电压是否新鲜且在协议范围内。
    bool feedback_fresh; // 四轮反馈是否全部新鲜。
    int16_t feedback_raw[CHASSIS_POWER_WHEEL_COUNT]; // 四轮反馈电流原值。
    int16_t output_raw[CHASSIS_POWER_WHEEL_COUNT]; // 准备发送的四路电流指令。
    float power_w; // 超电直接上报的功率，按已确认的原值单位 W。
    float target_w; // 扣除余量后的功率闭环目标。
    float current_scale; // 本周期四轮统一缩放比例。
    bool power_feedback_valid; // 超电功率帧是否新鲜，与电压有效性独立。
    bool limited; // 本周期是否减小了电流。
    bool control_config_valid; // 功率闭环配置是否有效。
    uint32_t power_sample_count; // 超电状态帧计数，区分新样本与重复读取。
    uint32_t power_rx_ms; // 超电功率帧的接收时间。
    int16_t request_raw[CHASSIS_POWER_WHEEL_COUNT]; // 功率限流前的四轮指令。
    uint32_t last_update_ms; // 最近一次观测更新时间。
} ChassisPowerState;

extern volatile ChassisPowerState chassis_power_state;

void ChassisPower_Init(void);
// 四轮电流发送前调用；按超电实测功率统一缩放。
float ChassisPower_Apply(int16_t currents[CHASSIS_POWER_WHEEL_COUNT]);
// 裁判动态上限供超电控制帧使用，输出关闭时返回零。
uint16_t ChassisPower_GetLimit(uint32_t now_ms, bool *output_allowed);
// 只读取最终电流指令并更新观察值，不改指令或闭环状态。
void ChassisPower_Update(const int16_t currents[CHASSIS_POWER_WHEEL_COUNT]);

#endif
