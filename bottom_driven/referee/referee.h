#ifndef REFEREE_H
#define REFEREE_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "referee_wire.h"

// 核心消息按已核对的 V1.3/V2.0 长度解析；此开关仅控制模板雷达/视频扩展。
#ifndef REFEREE_TEMPLATE_LAYOUT
#define REFEREE_TEMPLATE_LAYOUT 0 // 是否启用样例雷达/视频扩展格式。
#endif
#define REFEREE_MAX_PAYLOAD 300U // 裁判数据段最大长度，字节。
#define REFEREE_FRAME_CAPACITY (REFEREE_MAX_PAYLOAD + 9U) // 数据段加帧头、命令字和 CRC 的总容量。
#define REFEREE_OFFLINE_TIMEOUT_MS 1000U // 裁判有效帧及各消息过期时间，ms。
#define REFEREE_PARTIAL_TIMEOUT_MS 100U // 残缺帧等待超时，ms。

typedef enum {
    REFEREE_MSG_game_status = 0, // 0x0001
    REFEREE_MSG_game_result = 1, // 0x0002
    REFEREE_MSG_game_robot_HP = 2, // 0x0003
    REFEREE_MSG_event_data = 3, // 0x0101
    REFEREE_MSG_referee_warning = 4, // 0x0104
    REFEREE_MSG_dart_info = 5, // 0x0105
    REFEREE_MSG_robot_status = 6, // 0x0201
    REFEREE_MSG_power_heat_data = 7, // 0x0202
    REFEREE_MSG_robot_pos = 8, // 0x0203
    REFEREE_MSG_buff = 9, // 0x0204
    REFEREE_MSG_hurt_data = 10, // 0x0206
    REFEREE_MSG_shoot_data = 11, // 0x0207
    REFEREE_MSG_projectile_allowance = 12, // 0x0208
    REFEREE_MSG_rfid_status = 13, // 0x0209
    REFEREE_MSG_robot_interaction_data = 14, // 0x0301
    REFEREE_MSG_map_command = 15, // 0x0303
    REFEREE_MSG_map_robot_data = 16, // 0x0305
    REFEREE_MSG_map_data = 17, // 0x0307
    REFEREE_MSG_custom_info = 18, // 0x0308
    REFEREE_MSG_set_video_channel = 19, // 0x0F01
    REFEREE_MSG_query_video_channel = 20, // 0x0F02
    REFEREE_MSG_radar_enemy_HP = 21, // 0x0A02
    REFEREE_MSG_radar_enemy_ammo = 22, // 0x0A03
    REFEREE_MSG_radar_enemy_team_status = 23, // 0x0A04
    REFEREE_MSG_radar_enemy_robot_status = 24, // 0x0A05
    REFEREE_MSG_COUNT = 25
} RefereeMessage_t;

typedef struct {
    uint8_t game_type; // 比赛类型。
    uint8_t game_progress; // 0未开始/1准备/2自检/3倒计时/4比赛中/5结算。
    uint16_t stage_remain_time; // 本阶段剩余秒数。
    uint64_t SyncTimeStamp; // 裁判系统时间戳。
} RefereeGameStatus_t;
typedef struct {
    uint8_t robot_id; // 本机器人ID。
    uint8_t robot_level; // 等级。
    uint16_t current_HP; // 当前血量。
    uint16_t maximum_HP; // 血量上限。
    uint16_t shooter_barrel_cooling_value; // 每秒热量冷却值。
    uint16_t shooter_barrel_heat_limit; // 枪管热量上限。
    uint16_t chassis_power_limit; // 裁判系统下发的底盘功率上限，W。
    float shooter_barrel_speed_limit; // V2.0的17字节状态帧包含此字段；使用前检查speed_limit_valid。
    bool speed_limit_valid; // 旧版13字节状态帧没有初速度上限字段。
    uint8_t power_management_gimbal_output; // 云台电源输出许可，0/1。
    uint8_t power_management_chassis_output; // 底盘电源输出许可，0/1。
    uint8_t power_management_shooter_output; // 发射电源输出许可，0/1。
} RefereeRobotStatus_t;
typedef struct { uint8_t armor_id; uint8_t HP_deduction_reason; } RefereeHurtData_t;
typedef struct {
    bool valid; // 是否曾收到此命令的有效且可解析负载。
    bool fresh; // 消息已在配置的超时窗口内更新；低频事件请使用valid和更新时间。
    uint16_t payload_length; // 最近成功解析的数据段字节数。
    uint32_t last_rx_ms; // 此命令最后成功更新时间。
    uint32_t rx_count; // 此命令成功解析计数。
    uint32_t received_frame_count; // 此命令通过整帧 CRC 的次数，包含布局不支持的帧。
    uint16_t received_payload_length; // 此命令最近收到的 CRC 正确负载长度。

} RefereeMessageStatus_t;

typedef struct {
    RefereeGameStatus_t game_status; // 命令0x0001解析结果。
    RefereeWire_game_result_t game_result; // 命令0x0002解析结果。
    RefereeWire_game_robot_HP_t game_robot_HP; // 命令0x0003解析结果。
    RefereeWire_event_data_t event_data; // 命令0x0101解析结果。
    RefereeWire_referee_warning_t referee_warning; // 命令0x0104解析结果。
    RefereeWire_dart_info_t dart_info; // 命令0x0105解析结果。
    RefereeRobotStatus_t robot_status; // 命令0x0201解析结果。
    RefereeWire_power_heat_data_t power_heat_data; // 命令0x0202解析结果。
    RefereeWire_robot_pos_t robot_pos; // 命令0x0203解析结果。
    RefereeWire_buff_t buff; // 命令0x0204解析结果。
    RefereeHurtData_t hurt_data; // 命令0x0206解析结果。
    RefereeWire_shoot_data_t shoot_data; // 命令0x0207解析结果。
    RefereeWire_projectile_allowance_t projectile_allowance; // 命令0x0208解析结果。
    RefereeWire_rfid_status_t rfid_status; // 命令0x0209解析结果。
    RefereeWire_robot_interaction_data_t robot_interaction_data; // 命令0x0301解析结果。
    RefereeWire_map_command_t map_command; // 命令0x0303解析结果。
    RefereeWire_map_robot_data_t map_robot_data; // 命令0x0305解析结果。
    RefereeWire_map_data_t map_data; // 命令0x0307解析结果。
    RefereeWire_custom_info_t custom_info; // 命令0x0308解析结果。
    RefereeWire_set_video_channel_t set_video_channel; // 命令0x0F01解析结果。
    RefereeWire_query_video_channel_t query_video_channel; // 命令0x0F02解析结果。
    RefereeWire_radar_enemy_HP_t radar_enemy_HP; // 命令0x0A02解析结果。
    RefereeWire_radar_enemy_ammo_t radar_enemy_ammo; // 命令0x0A03解析结果。
    RefereeWire_radar_enemy_team_status_t radar_enemy_team_status; // 命令0x0A04解析结果。
    RefereeWire_radar_enemy_robot_status_t radar_enemy_robot_status; // 命令0x0A05解析结果。
    uint16_t interaction_user_length; // 0x0301实际user_data长度，尾部已清零。
} RefereeInfo_t;
typedef struct {
    bool online; // 最近收到CRC正确的完整帧；不表示功率数据一定新鲜。
    uint8_t last_seq;
    uint16_t last_cmd_id;
    uint16_t last_payload_length;
    uint32_t last_rx_ms;
    uint32_t valid_frame_count;
    uint32_t parsed_frame_count;
    uint32_t crc8_error_count;
    uint32_t crc16_error_count;
    uint32_t length_error_count;
    uint32_t unknown_command_count;
    uint32_t discarded_byte_count;
    uint32_t partial_timeout_count;
    uint32_t stream_gap_count;
    uint8_t last_payload[REFEREE_MAX_PAYLOAD]; // 包括尚未支持或长度不符的有效CRC负载。
} RefereeDiagnostics_t;
typedef struct {
    RefereeInfo_t info; // 各类解析数据，保留最后有效值。
    RefereeMessageStatus_t message[REFEREE_MSG_COUNT]; // 按枚举查看有效性与更新时刻。
    RefereeDiagnostics_t diagnostics;
} RefereeState_t;
extern volatile RefereeState_t referee_state; // Keil可直接展开；控制代码用快照接口。

void Referee_Init(void); // 启动接收前调用一次。
void Referee_Feed(const uint8_t *data, size_t length, uint32_t now_ms); // 仅同一个解析任务调用，可传半帧/粘包。
void Referee_Update(uint32_t now_ms); // 周期更新在线、新鲜度以及未完成帧超时。
void Referee_ResetStream(void); // 字节丢失后丢弃未完成帧，保留最后有效数据。
bool Referee_GetState(RefereeState_t *state); // 非阻塞一致快照；写入时返回false，可下次再读。
bool Referee_GetRobotStatusSnapshot(RefereeRobotStatus_t *status, uint32_t *last_rx_ms); // 一致读取最后有效状态及原始接收时间。
bool Referee_GetRobotStatus(RefereeRobotStatus_t *status, uint32_t now_ms); // 只返回新鲜机器人状态。
bool Referee_GetPowerHeat(RefereeWire_power_heat_data_t *data, uint32_t now_ms); // 只返回新鲜热量/缓冲数据。
typedef struct {
    uint16_t heat, limit, cooling; // 当前热量、热量上限、每秒冷却量。
    uint8_t sequence; // 热量消息更新序号，转发同一数据时保持不变。
    bool valid, output_allowed; // 两类消息均新鲜、发射电源许可。
} RefereeHeatSnapshot_t;
bool Referee_GetHeatSnapshot(RefereeHeatSnapshot_t *heat, uint32_t now_ms);
int Referee_CommandIndex(uint16_t cmd_id);
uint8_t Referee_Crc8(const uint8_t *data, size_t length);
uint16_t Referee_Crc16(const uint8_t *data, size_t length);
#endif
