#ifndef REFEREE_WIRE_H
#define REFEREE_WIRE_H
#include <stdint.h>
// 来源：双板样例下板 Application/DeviceLayer/judge.h；只保留协议负载类型。
// 这些是模板 wire 格式；版本差异由 referee.c 明确处理，不能直接当成当前串口帧。
#define REFEREE_PACKED __attribute__((packed)) // 按协议紧凑排列，去掉结构体填充。

typedef struct REFEREE_PACKED
{
    uint8_t  game_type     : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} RefereeWire_game_status_t;
typedef struct REFEREE_PACKED
{
    uint8_t winner;
} RefereeWire_game_result_t;
typedef struct REFEREE_PACKED
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    int16_t damage_difference; // V2.0为双方伤害差；旧版保留位解析置零。
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
    uint16_t enemy_outpost_HP; // V2.0的20字节格式包含此字段，旧版置零。
    uint16_t enemy_base_HP; // V2.0的20字节格式包含此字段，旧版置零。
} RefereeWire_game_robot_HP_t;
typedef struct REFEREE_PACKED
{
    uint32_t event_data;
} RefereeWire_event_data_t;
typedef struct REFEREE_PACKED
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} RefereeWire_referee_warning_t;
typedef struct REFEREE_PACKED
{
    uint8_t  dart_remaining_time;
    uint16_t dart_info;
} RefereeWire_dart_info_t;
typedef struct REFEREE_PACKED
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    float shooter_barrel_speed_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved                        : 5;
} RefereeWire_robot_status_t;
typedef struct REFEREE_PACKED
{
    uint16_t reserved1; // 协议保留位，不作为底盘电压。
    uint16_t reserved2; // 协议保留位，不作为底盘电流。
    float    reserved3; // 协议保留位，不作为实时底盘功率。
    uint16_t buffer_energy; // 底盘缓冲能量，J。
    uint16_t shooter_17mm_1_barrel_heat; // 17mm枪管当前热量。
    uint16_t shooter_42mm_barrel_heat; // 42mm枪管当前热量。
} RefereeWire_power_heat_data_t;
typedef struct REFEREE_PACKED
{
    float x;
    float y;
    float angle;
} RefereeWire_robot_pos_t;
typedef struct REFEREE_PACKED
{
    uint8_t  recovery_buff;
    uint16_t cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
    uint8_t  remaining_energy;
} RefereeWire_buff_t;
typedef struct REFEREE_PACKED
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} RefereeWire_hurt_data_t;
typedef struct REFEREE_PACKED
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency; // 发射频率，Hz。
    float   initial_speed; // 最近一发弹丸初速度，m/s。
} RefereeWire_shoot_data_t;
typedef struct REFEREE_PACKED
{
    uint16_t projectile_allowance_17mm; // 17mm剩余允许发弹量。
    uint16_t projectile_allowance_42mm; // 42mm剩余允许发弹量。
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} RefereeWire_projectile_allowance_t;
typedef struct REFEREE_PACKED
{
    uint32_t rfid_status;
    uint8_t  rfid_status_2;
} RefereeWire_rfid_status_t;
typedef struct REFEREE_PACKED
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t  user_data[112];
} RefereeWire_robot_interaction_data_t;
typedef struct REFEREE_PACKED
{
    float    target_position_x;
    float    target_position_y;
    uint8_t  cmd_keyboard;
    uint8_t  target_robot_id;
    uint16_t cmd_source;
} RefereeWire_map_command_t;
typedef struct REFEREE_PACKED
{
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
} RefereeWire_map_robot_data_t;
typedef struct REFEREE_PACKED
{
    uint8_t  intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t   delta_x[49];
    int8_t   delta_y[49];
    uint16_t sender_id;
} RefereeWire_map_data_t;
typedef struct REFEREE_PACKED
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t  user_data[30];
} RefereeWire_custom_info_t;
typedef struct REFEREE_PACKED
{
    uint8_t channel;
} RefereeWire_set_video_channel_t;
typedef struct REFEREE_PACKED
{
    uint8_t query;
} RefereeWire_query_video_channel_t;
typedef struct REFEREE_PACKED
{
    uint16_t enemy_hero_HP;
    uint16_t enemy_engineer_HP;
    uint16_t enemy_infantry_3_HP;
    uint16_t enemy_infantry_4_HP;
    uint16_t enemy_sentry_HP;
		uint32_t update_timestamp;
} RefereeWire_radar_enemy_HP_t;
typedef struct REFEREE_PACKED
{
    uint16_t enemy_hero_ammo;
    uint16_t enemy_infantry_3_ammo;
    uint16_t enemy_infantry_4_ammo;
    uint16_t enemy_aerial_ammo;
    uint16_t enemy_sentry_ammo;
	  uint32_t update_timestamp;
} RefereeWire_radar_enemy_ammo_t;
typedef struct REFEREE_PACKED
{
    uint16_t remaining_gold_coin;
    uint16_t total_gold_coin;
    uint32_t field_status;
	  uint32_t update_timestamp;
} RefereeWire_radar_enemy_team_status_t;
typedef struct REFEREE_PACKED
{
    uint8_t  recovery_buff;
    uint16_t cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
    uint16_t attack_buff;
} RefereeWire_radar_enemy_robot_buff_t;
typedef struct REFEREE_PACKED
{
    RefereeWire_radar_enemy_robot_buff_t hero;
    RefereeWire_radar_enemy_robot_buff_t engineer;
    RefereeWire_radar_enemy_robot_buff_t infantry_3;
    RefereeWire_radar_enemy_robot_buff_t infantry_4;
    RefereeWire_radar_enemy_robot_buff_t sentry;
    uint8_t sentry_mode;
    uint8_t hero_status;
    uint8_t engineer_status;
    uint8_t infantry_3_status;
    uint8_t infantry_4_status;
    uint8_t sentry_status;
		uint32_t update_timestamp;
} RefereeWire_radar_enemy_robot_status_t;
typedef struct REFEREE_PACKED
{
    RefereeWire_radar_enemy_HP_t            radar_enemy_HP;
    RefereeWire_radar_enemy_ammo_t          radar_enemy_ammo;
    RefereeWire_radar_enemy_team_status_t   radar_enemy_team_status;
    RefereeWire_radar_enemy_robot_status_t  radar_enemy_robot_status;
	  uint32_t current_timestamp;
} RefereeWire_radar_information_status_t;

#endif
