#ifndef SHOOT_SPEED_H
#define SHOOT_SPEED_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled; // 弹速自适应开关。
    int32_t step_rpm; // 每次有效反馈调整的转速幅值。
    int32_t max_speed_rpm; // 正常速度控制的转速上限，rpm。
    float margin_m_s; // 目标弹速与弹速上限的余量，m/s。
    float deadband_m_s; // 目标弹速附近的保持范围，m/s。
    float fallback_limit_m_s; // 裁判未提供弹速上限时使用的上限。
    uint32_t feedback_timeout_ms; // 板间弹速帧有效时间。
    uint32_t settle_time_ms; // 调速后等待摩擦轮稳定的时间。
} ShootSpeedConfig;

typedef struct {
    float actual_m_s, limit_m_s; // 本发实测弹速、裁判弹速上限。
    uint16_t sequence; // 每发更新一次，转发时保持不变。
    bool valid, limit_valid; // 弹速和上限各自的有效标志。
    uint32_t received_ms, sample_ms; // 板间接收时刻、折算到本板的测速时刻。
} ShootSpeedFeedback;

typedef struct {
    int32_t target_speed_rpm; // 正常控制实际使用的摩擦轮转速幅值。
    float actual_m_s, target_m_s, limit_m_s; // 实测、目标弹速及采用的弹速上限。
    uint32_t adjusted_count; // 已执行的转速调整次数。
    uint32_t stable_since_ms; // 当前转速和学习许可开始保持的时刻。
    uint16_t sequence; // 已处理的弹速序号。
    bool sequence_seen; // 已消费过测速序号。
    bool learning; // 当前状态允许根据弹速调整。
    bool feedback_valid; // 最新测速在有效时间内。
    bool config_valid; // 调速参数通过范围检查。
    int32_t manual_speed_rpm; // 上次读取的基础转速，手动修改后重新起调。
    bool initialized; // 已建立基础转速。
} ShootSpeedState;

void ShootSpeed_Init(volatile ShootSpeedState *state); // 清空自适应状态。
void ShootSpeed_Update(volatile ShootSpeedState *state, const ShootSpeedConfig *config,
    int32_t base_rpm, float driver_max_rpm, const ShootSpeedFeedback *feedback,
    bool ready, uint32_t now_ms); // 新测速反馈按步长修正正常目标转速。
#endif
