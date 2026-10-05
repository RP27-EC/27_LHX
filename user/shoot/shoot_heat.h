#ifndef SHOOT_HEAT_H
#define SHOOT_HEAT_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool enabled; // 热量限制开关。
    bool allow_offline; // 无裁判热量时允许使用备用参数调试。
    float heat_per_shot; // 每次供弹预留的热量。
    float stop_remaining; // 剩余热量不超过此值时禁止新供弹。
    float low_remaining; // 低速连发档的剩余热量上界，含边界。
    float high_remaining; // 高速连发档的剩余热量下界，含边界。
    float low_rate_hz; // 低余量连发射速，发/秒。
    float middle_rate_hz; // 中余量连发射速，发/秒。
    float high_rate_hz; // 高余量连发射速，发/秒。
    float offline_heat_limit; // 调试备用热量上限。
    float offline_cooling_per_s; // 调试备用冷却速率。
    uint32_t referee_timeout_ms; // 上板热量报文的有效时间。
    uint32_t calibration_settle_ms; // 停止供弹后，允许向下校准前的等待时间。
} ShootHeatConfig;

typedef struct
{
    float predicted_heat; // 本地预测热量。
    float referee_heat; // 最近收到的裁判当前热量。
    float heat_limit; // 当前生效的热量上限。
    float cooling_per_s; // 当前生效的每秒冷却值。
    float remaining; // 可用剩余热量。
    float continuous_rate_hz; // 当前允许的连发速度。
    bool referee_valid; // 裁判热量与参数是否有效。
    bool output_allowed; // 裁判发射许可。
    bool config_valid; // 参数检查结果。
    bool initialized; // 是否已建立热量初值。
    bool sample_seen; // 已接收过校准样本。
    uint8_t sample_sequence; // 已处理的热量样本序号。
    uint32_t last_update_ms; // 最近冷却计算时间。
    uint32_t last_feed_ms; // 最近供弹活动或预留热量的时间。
    uint32_t predicted_shots; // 本地已预留的累计发数。
    uint32_t calibration_count; // 新裁判样本的校准次数。
} ShootHeatState;

extern volatile ShootHeatState shoot_heat_state;
void ShootHeat_Init(uint32_t now_ms);
// 每周期先冷却；射击中仅向上校准，停射等待后同步裁判值。
void ShootHeat_Update(const ShootHeatConfig *config, uint32_t now_ms,
                      bool valid, bool output_allowed, uint8_t sequence,
                      float heat, float limit, float cooling, bool feeding);
bool ShootHeat_CanStart(const ShootHeatConfig *config);
void ShootHeat_RecordShot(const ShootHeatConfig *config, uint32_t now_ms);
float ShootHeat_GetRate(const ShootHeatConfig *config);
#endif
