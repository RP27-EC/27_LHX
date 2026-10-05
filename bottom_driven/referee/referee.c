#include "referee.h"
#include <string.h>

volatile RefereeState_t referee_state; // 已解析数据及消息有效性。
static volatile uint32_t publish_version; // 奇数表示正在写，偶数表示可读取。
static uint8_t frame[REFEREE_FRAME_CAPACITY]; // 正在拼接的协议帧。
static uint16_t buffered; // 当前缓存的字节数。
static uint32_t partial_last_byte_ms; // 未完成帧最近补入字节的时间。
static const uint16_t command_ids[REFEREE_MSG_COUNT] = {
0x0001U,0x0002U,0x0003U,0x0101U,0x0104U,0x0105U,0x0201U,0x0202U,0x0203U,0x0204U,0x0206U,0x0207U,0x0208U,0x0209U,0x0301U,0x0303U,0x0305U,0x0307U,0x0308U,0x0F01U,0x0F02U,0x0A02U,0x0A03U,0x0A04U,0x0A05U};

static uint16_t read_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint64_t read_u64(const uint8_t *p) {
    uint64_t result = 0; uint8_t i;
    for (i=0;i<8U;i++) { result |= (uint64_t)p[i] << (8U*i); }
    return result;
}
// 计算裁判帧头 CRC8。
uint8_t Referee_Crc8(const uint8_t *data, size_t length) {
    uint8_t crc=0xFFU, bit; size_t i;
    for (i=0;i<length;i++) { crc ^= data[i]; for(bit=0;bit<8U;bit++) { crc=(uint8_t)((crc>>1)^((crc&1U)?0x8CU:0U)); } }
    return crc;
}
// 计算完整裁判帧 CRC16。
uint16_t Referee_Crc16(const uint8_t *data, size_t length) {
    uint16_t crc=0xFFFFU; uint8_t bit; size_t i;
    for(i=0;i<length;i++) { crc ^= data[i]; for(bit=0;bit<8U;bit++) { crc=(uint16_t)((crc>>1)^((crc&1U)?0x8408U:0U)); } }
    return crc;
}
// 把命令字映射到消息状态索引。
int Referee_CommandIndex(uint16_t cmd_id) {
    uint8_t i; for(i=0;i<REFEREE_MSG_COUNT;i++) { if(command_ids[i]==cmd_id) { return i; } } return -1;
}
// 每个分支先检查真实数据段长度；不能把结构体sizeof当成接收缓冲区长度。
static bool decode(uint16_t command, const uint8_t *p, uint16_t length) {
    switch(command) {
    case 0x0001U:
        if(length!=11U) { return false; }
        referee_state.info.game_status.game_type=p[0]&15U;
        referee_state.info.game_status.game_progress=p[0]>>4;
        referee_state.info.game_status.stage_remain_time=read_u16(p+1);
        referee_state.info.game_status.SyncTimeStamp=read_u64(p+3); return true;
    case 0x0003U:
        if(length!=16U && length!=20U) { return false; }
        memset((void *)&referee_state.info.game_robot_HP,0,sizeof(referee_state.info.game_robot_HP));
        memcpy((void *)&referee_state.info.game_robot_HP,p,length);
if (length==16U) { referee_state.info.game_robot_HP.damage_difference=0; } // 旧版此处为保留位。
        return true;
    case 0x0201U: {
        uint8_t flags;
        if(length!=13U && length!=17U) { return false; }
        referee_state.info.robot_status.robot_id=p[0]; referee_state.info.robot_status.robot_level=p[1];
        referee_state.info.robot_status.current_HP=read_u16(p+2); referee_state.info.robot_status.maximum_HP=read_u16(p+4);
        referee_state.info.robot_status.shooter_barrel_cooling_value=read_u16(p+6);
        referee_state.info.robot_status.shooter_barrel_heat_limit=read_u16(p+8);
        referee_state.info.robot_status.chassis_power_limit=read_u16(p+10);
        referee_state.info.robot_status.speed_limit_valid=length==17U;
        referee_state.info.robot_status.shooter_barrel_speed_limit=0.0f;
if (length==17U) { memcpy((void *)&referee_state.info.robot_status.shooter_barrel_speed_limit,p+12,4U); }
        flags=p[length==17U?16U:12U];
        referee_state.info.robot_status.power_management_gimbal_output=flags&1U;
        referee_state.info.robot_status.power_management_chassis_output=(flags>>1)&1U;
        referee_state.info.robot_status.power_management_shooter_output=(flags>>2)&1U;
        return true;
    }
    case 0x0206U:
        if(length!=1U) { return false; }
        referee_state.info.hurt_data.armor_id=p[0]&15U;
        referee_state.info.hurt_data.HP_deduction_reason=p[0]>>4; return true;
    case 0x0301U:
        if(length<6U || length>sizeof(RefereeWire_robot_interaction_data_t)) { return false; }
        memset((void *)&referee_state.info.robot_interaction_data,0,sizeof(referee_state.info.robot_interaction_data));
        memcpy((void *)&referee_state.info.robot_interaction_data,p,length);
        referee_state.info.interaction_user_length=length-6U; return true;
    case 0x0002U:
        if(length!=sizeof(referee_state.info.game_result)) { return false; }
        memcpy((void *)&referee_state.info.game_result,p,length); return true;
    case 0x0101U:
        if(length!=sizeof(referee_state.info.event_data)) { return false; }
        memcpy((void *)&referee_state.info.event_data,p,length); return true;
    case 0x0104U:
        if(length!=sizeof(referee_state.info.referee_warning)) { return false; }
        memcpy((void *)&referee_state.info.referee_warning,p,length); return true;
    case 0x0105U:
        if(length!=sizeof(referee_state.info.dart_info)) { return false; }
        memcpy((void *)&referee_state.info.dart_info,p,length); return true;
    case 0x0202U:
        if(length!=sizeof(referee_state.info.power_heat_data)) { return false; }
        memcpy((void *)&referee_state.info.power_heat_data,p,length); return true;
    case 0x0203U:
        if(length!=sizeof(referee_state.info.robot_pos)) { return false; }
        memcpy((void *)&referee_state.info.robot_pos,p,length); return true;
    case 0x0204U:
        if(length!=sizeof(referee_state.info.buff)) { return false; }
        memcpy((void *)&referee_state.info.buff,p,length); return true;
    case 0x0207U:
        if(length!=sizeof(referee_state.info.shoot_data)) { return false; }
        memcpy((void *)&referee_state.info.shoot_data,p,length); return true;
    case 0x0208U:
        if(length!=sizeof(referee_state.info.projectile_allowance)) { return false; }
        memcpy((void *)&referee_state.info.projectile_allowance,p,length); return true;
    case 0x0209U:
        if(length!=sizeof(referee_state.info.rfid_status)) { return false; }
        memcpy((void *)&referee_state.info.rfid_status,p,length); return true;
    case 0x0303U:
        if(length!=sizeof(referee_state.info.map_command)) { return false; }
        memcpy((void *)&referee_state.info.map_command,p,length); return true;
    case 0x0305U:
        if(length!=sizeof(referee_state.info.map_robot_data)) { return false; }
        memcpy((void *)&referee_state.info.map_robot_data,p,length); return true;
    case 0x0307U:
        if(length!=sizeof(referee_state.info.map_data)) { return false; }
        memcpy((void *)&referee_state.info.map_data,p,length); return true;
    case 0x0308U:
        if(length!=sizeof(referee_state.info.custom_info)) { return false; }
        memcpy((void *)&referee_state.info.custom_info,p,length); return true;
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0F01U:
        if(length!=sizeof(referee_state.info.set_video_channel)) { return false; }
        memcpy((void *)&referee_state.info.set_video_channel,p,length); return true;
#endif
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0F02U:
        if(length!=sizeof(referee_state.info.query_video_channel)) { return false; }
        memcpy((void *)&referee_state.info.query_video_channel,p,length); return true;
#endif
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0A02U:
        if(length!=sizeof(referee_state.info.radar_enemy_HP)) { return false; }
        memcpy((void *)&referee_state.info.radar_enemy_HP,p,length); return true;
#endif
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0A03U:
        if(length!=sizeof(referee_state.info.radar_enemy_ammo)) { return false; }
        memcpy((void *)&referee_state.info.radar_enemy_ammo,p,length); return true;
#endif
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0A04U:
        if(length!=sizeof(referee_state.info.radar_enemy_team_status)) { return false; }
        memcpy((void *)&referee_state.info.radar_enemy_team_status,p,length); return true;
#endif
#if REFEREE_TEMPLATE_LAYOUT
    case 0x0A05U:
        if(length!=sizeof(referee_state.info.radar_enemy_robot_status)) { return false; }
        memcpy((void *)&referee_state.info.radar_enemy_robot_status,p,length); return true;
#endif
    default: return false;
    }
}
// 丢掉已处理或损坏字节，保留后续数据。
static void discard(uint16_t count) {
    if(count>buffered) { count=buffered; }
    buffered-=count; if(buffered) { memmove(frame,frame+count,buffered); }
}
// 有效帧按数据段长度取出；损坏帧只丢一个字节再同步，不递归。
static void parse(uint32_t now_ms) {
    while(buffered) {
        uint16_t payload_length,total,command; int index; bool parsed;
        if(frame[0]!=0xA5U) { referee_state.diagnostics.discarded_byte_count++; discard(1U); continue; }
        // 帧头不足先等后续字节，头 CRC 正确后才信任长度。
        if(buffered<5U) { return; }
        if(Referee_Crc8(frame,4U)!=frame[4]) { referee_state.diagnostics.crc8_error_count++; discard(1U); continue; }
        payload_length=read_u16(frame+1);
        if(payload_length>REFEREE_MAX_PAYLOAD) { referee_state.diagnostics.length_error_count++; discard(1U); continue; }
        total=payload_length+9U; if(buffered<total) { return; }
        if(Referee_Crc16(frame,total-2U)!=read_u16(frame+total-2U)) { referee_state.diagnostics.crc16_error_count++; discard(1U); continue; }
        command=read_u16(frame+5); index=Referee_CommandIndex(command);
#if !REFEREE_TEMPLATE_LAYOUT
        if(command>=0x0A00U) { index=-1; } // 模板雷达/视频私有格式只保留原始负载。
#endif
        referee_state.diagnostics.online=true; referee_state.diagnostics.last_rx_ms=now_ms;
        referee_state.diagnostics.last_seq=frame[3]; referee_state.diagnostics.last_cmd_id=command;
        referee_state.diagnostics.last_payload_length=payload_length; referee_state.diagnostics.valid_frame_count++;
        memset((void *)referee_state.diagnostics.last_payload,0,REFEREE_MAX_PAYLOAD);
        memcpy((void *)referee_state.diagnostics.last_payload,frame+7,payload_length);
        if (index>=0) {
            referee_state.message[index].received_frame_count++;
            referee_state.message[index].received_payload_length=payload_length;
        }
        // 整帧 CRC 校验后，decode 再按命令核对负载长度。
        parsed=decode(command,frame+7,payload_length);
        if(parsed && index>=0) {
            referee_state.message[index].valid=true; referee_state.message[index].fresh=true;
            referee_state.message[index].payload_length=payload_length; referee_state.message[index].last_rx_ms=now_ms;
            referee_state.message[index].rx_count++; referee_state.diagnostics.parsed_frame_count++;
        } else if(index>=0) { referee_state.diagnostics.length_error_count++; }
        else { referee_state.diagnostics.unknown_command_count++; }
        discard(total);
    }
}
// 清空裁判结构体、字节缓存和发布版本。
void Referee_Init(void) {
    memset((void *)&referee_state,0,sizeof(referee_state)); buffered=0U;
    partial_last_byte_ms=0U; publish_version=0U;
}
// 丢弃不连续的半帧，保留已解析消息。
void Referee_ResetStream(void) {
    publish_version++; buffered=0U; referee_state.diagnostics.stream_gap_count++; publish_version++;
}
// 更新在线和新鲜标志，超时半帧重新找帧头。
void Referee_Update(uint32_t now_ms) {
    uint8_t i; publish_version++;
    referee_state.diagnostics.online=referee_state.diagnostics.valid_frame_count!=0U &&
        (uint32_t)(now_ms-referee_state.diagnostics.last_rx_ms)<REFEREE_OFFLINE_TIMEOUT_MS;
    for(i=0;i<REFEREE_MSG_COUNT;i++) { referee_state.message[i].fresh=referee_state.message[i].valid &&
        (uint32_t)(now_ms-referee_state.message[i].last_rx_ms)<REFEREE_OFFLINE_TIMEOUT_MS; }
    if(buffered && (uint32_t)(now_ms-partial_last_byte_ms)>=REFEREE_PARTIAL_TIMEOUT_MS) {
        discard(1U); referee_state.diagnostics.partial_timeout_count++; parse(now_ms);
        partial_last_byte_ms=now_ms; // 尝试救回错误长度后夹带的完整帧。
    }
    publish_version++;
}
// 逐字节拼接，长度和 CRC 都正确才发布消息。
void Referee_Feed(const uint8_t *data, size_t length, uint32_t now_ms) {
    size_t i; if(data==NULL || length==0U) { return; }
    Referee_Update(now_ms); publish_version++;
    for(i=0;i<length;i++) {
        if(buffered>=REFEREE_FRAME_CAPACITY) { discard(1U); referee_state.diagnostics.discarded_byte_count++; }
        frame[buffered++]=data[i]; partial_last_byte_ms=now_ms; parse(now_ms);
    }
    publish_version++;
}
// 读前后检查版本，避免复制到一半更新的消息。
static bool snapshot(void *out, const volatile void *source, size_t size) {
    uint8_t attempt; size_t i; uint32_t version;
    if(out==NULL) { return false; }
    for(attempt=0;attempt<3U;attempt++) {
        version=publish_version; if(version&1U) { return false; }
        for(i=0;i<size;i++) { ((uint8_t *)out)[i]=((const volatile uint8_t *)source)[i]; }
        if(version==publish_version) { return true; }
    }
    return false;
}
bool Referee_GetState(RefereeState_t *state) { return snapshot(state,&referee_state,sizeof(*state)); }
// 不刷新接收时间；过期状态仍可用于保留底盘断电许可。
bool Referee_GetRobotStatusSnapshot(RefereeRobotStatus_t *status, uint32_t *last_rx_ms) {
    RefereeMessageStatus_t meta; uint32_t version=publish_version;
    if(status==NULL || last_rx_ms==NULL || (version&1U)) { return false; }
    if(!snapshot(&meta,&referee_state.message[REFEREE_MSG_robot_status],sizeof(meta)) || !meta.valid) { return false; }
    if(!snapshot(status,&referee_state.info.robot_status,sizeof(*status)) || version!=publish_version) { return false; }
    *last_rx_ms=meta.last_rx_ms; return true;
}
bool Referee_GetRobotStatus(RefereeRobotStatus_t *status,uint32_t now_ms) {
    uint32_t last_rx_ms;
    return Referee_GetRobotStatusSnapshot(status,&last_rx_ms) &&
           (uint32_t)(now_ms-last_rx_ms)<REFEREE_OFFLINE_TIMEOUT_MS;
}
bool Referee_GetPowerHeat(RefereeWire_power_heat_data_t *data,uint32_t now_ms) {
    RefereeMessageStatus_t meta; uint32_t version=publish_version;
    if(!snapshot(&meta,&referee_state.message[REFEREE_MSG_power_heat_data],sizeof(meta)) || !meta.valid ||
       (uint32_t)(now_ms-meta.last_rx_ms)>=REFEREE_OFFLINE_TIMEOUT_MS) { return false; }
    return snapshot(data,&referee_state.info.power_heat_data,sizeof(*data)) && version==publish_version;
}

bool Referee_GetHeatSnapshot(RefereeHeatSnapshot_t *heat, uint32_t now_ms)
{
    RefereeRobotStatus_t robot;
    RefereeWire_power_heat_data_t power;
    RefereeMessageStatus_t status_meta, heat_meta;
    uint32_t version = publish_version;
    if (heat == NULL) { return false; }
    memset(heat, 0, sizeof(*heat));
    if ((version & 1U) ||
        !snapshot(&robot, &referee_state.info.robot_status, sizeof(robot)) ||
        !snapshot(&power, &referee_state.info.power_heat_data, sizeof(power)) ||
        !snapshot(&status_meta, &referee_state.message[REFEREE_MSG_robot_status], sizeof(status_meta)) ||
        !snapshot(&heat_meta, &referee_state.message[REFEREE_MSG_power_heat_data], sizeof(heat_meta)) ||
        version != publish_version) { return false; }
    heat->heat = power.shooter_17mm_1_barrel_heat;
    heat->limit = robot.shooter_barrel_heat_limit;
    heat->cooling = robot.shooter_barrel_cooling_value;
    heat->sequence = (uint8_t)heat_meta.rx_count;
    heat->output_allowed = robot.power_management_shooter_output != 0U;
    heat->valid = status_meta.valid && heat_meta.valid && heat->limit > 0U &&
        (uint32_t)(now_ms - status_meta.last_rx_ms) < REFEREE_OFFLINE_TIMEOUT_MS &&
        (uint32_t)(now_ms - heat_meta.last_rx_ms) < REFEREE_OFFLINE_TIMEOUT_MS;
    return true;
}

// 绑定现有状态与函数，供外部通过模块结构体访问。
const RefereeModule referee =
{
    .data = {
        .state = &referee_state,
    },
    .init = Referee_Init,
    .feed = Referee_Feed,
    .update = Referee_Update,
    .reset_stream = Referee_ResetStream,
    .get_state = Referee_GetState,
    .get_robot_status_snapshot = Referee_GetRobotStatusSnapshot,
    .get_robot_status = Referee_GetRobotStatus,
    .get_power_heat = Referee_GetPowerHeat,
    .get_heat_snapshot = Referee_GetHeatSnapshot,
    .command_index = Referee_CommandIndex,
    .crc8 = Referee_Crc8,
    .crc16 = Referee_Crc16,
};
