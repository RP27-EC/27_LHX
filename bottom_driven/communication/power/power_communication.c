#include "power_communication.h"
#include "chassis_can.h"
#include "peripheral_config.h"
#include "chassis_power.h"
#include "referee.h"

#define CAP_BUFFER_MAX 250U // 超电接收端允许的缓冲能量上限，J。
#define CAP_DISCHARGE_LIMIT (-300) // 放电功率字段，符号按超电协议。
#define CAP_CHARGE_LIMIT 300U // 正常充电功率字段。

volatile PowerCommunicationState power_communication_state;

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static float scale_bipolar(int16_t raw, float maximum)
{
    // 协议将 -32000~32000 线性映射到 0~maximum。
    return ((float)raw + 32000.0f) * (maximum / 64000.0f);
}

static float scale_signed(int16_t raw, float maximum)
{
    // 电容电流的 -32000~32000 对应 -maximum~maximum。
    return (float)raw * (maximum / 32000.0f);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void PowerCommunication_SendControl(uint32_t now)
{
    uint8_t data[8] = {0};
    RefereeWire_power_heat_data_t power_heat;
    uint32_t i;
    bool output_allowed;
    uint16_t limit = ChassisPower_GetLimit(now, &output_allowed);

    // 逐字节构造控制帧，避免结构体对齐影响协议。
    // 读取本次发送时的裁判缓冲；未收到或过期时保持保守回退。
    if (referee.get_power_heat(&power_heat, now))
    {
        data[0] = (uint8_t)(power_heat.buffer_energy > CAP_BUFFER_MAX ?
                           CAP_BUFFER_MAX : power_heat.buffer_energy);
    }
    write_u16_le(&data[1], limit);
    write_u16_le(&data[3], (uint16_t)CAP_DISCHARGE_LIMIT);
    // 预充模式下清零充电功率字段。
    write_u16_le(&data[5], power_communication_config.precharge_enabled ?
                 0U : CAP_CHARGE_LIMIT);
    if (power_communication_config.cap_enabled && output_allowed) { data[7] |= 0x01U; }
    if (power_communication_config.turbo_enabled) { data[7] |= 0x02U; }
    if (power_communication_config.precharge_enabled) { data[7] |= 0x04U; }

    if (chassis_can.send(POWER_CAPACITOR_CONTROL_ID, data) == HAL_OK)
    {
        for (i = 0U; i < 8U; ++i)
        { power_communication_state.control.raw[i] = data[i]; }
        power_communication_state.control.last_tx_ms = now;
        power_communication_state.control.tx_count++;
    }
    else
    {
        power_communication_state.control.tx_error_count++;
    }
}

void PowerCommunication_ProcessCanFrame(uint32_t id, const uint8_t data[8])
{
    uint32_t i;
    uint32_t now = HAL_GetTick();

    if (data == NULL) { return; }
    if (id == POWER_CAPACITOR_STATUS_ID)
    {
        volatile CapacitorStatus *status = &power_communication_state.capacitor;
        for (i = 0U; i < 8U; ++i) { status->raw[i] = data[i]; }
        status->chassis_power_raw = read_i16_le(&data[0]);
        status->voltage_raw = read_i16_le(&data[2]);
        status->current_raw = read_i16_le(&data[4]);
        status->voltage_v = scale_bipolar(status->voltage_raw, 25.0f);
        status->current_a = scale_signed(status->current_raw, 16.0f);
        status->discharge_available = (data[6] & 0x01U) != 0U;
        status->precharge_active = (data[6] & 0x02U) != 0U;
        status->last_rx_ms = now;
        status->rx_count++;
        status->received = true;
        status->online = true;
    }
    else if (id == POWER_WIRELESS_STATUS_ID)
    {
        volatile WirelessChargeStatus *status = &power_communication_state.wireless;
        for (i = 0U; i < 8U; ++i) { status->raw[i] = data[i]; }
        status->charging_power_raw = read_i16_le(&data[0]);
        status->charging_power_w = scale_bipolar(status->charging_power_raw, 150.0f);
        status->charging = data[2] != 0U;
        status->last_rx_ms = now;
        status->rx_count++;
        status->received = true;
        status->online = true;
    }
}

// 清空超电、无线充解析结果及控制帧发送统计。
void PowerCommunication_Init(void)
{
    power_communication_state = (PowerCommunicationState){0};
}

void PowerCommunication_Service(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t timeout = power_communication_config.offline_timeout_ms;
    uint32_t period = power_communication_config.tx_period_ms;

    power_communication_state.capacitor.online =
        power_communication_state.capacitor.received &&
        ((uint32_t)(now - power_communication_state.capacitor.last_rx_ms) < timeout);
    power_communication_state.wireless.online =
        power_communication_state.wireless.received &&
        ((uint32_t)(now - power_communication_state.wireless.last_rx_ms) < timeout);
    if (period == 0U) { period = 1U; }
    if (power_communication_state.control.tx_count == 0U ||
        (uint32_t)(now - power_communication_state.control.last_tx_ms) >= period)
    { PowerCommunication_SendControl(now); }
}

bool PowerCommunication_GetSnapshot(PowerCommunicationState *snapshot)
{
    uint32_t saved_primask;
    if (snapshot == NULL) { return false; }
    saved_primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = power_communication_state;
    __set_PRIMASK(saved_primask);
    return true;
}

// 绑定现有状态与函数，供外部通过模块结构体访问。
const PowerLinkModule power_link =
{
    .config = &power_communication_config,
    .data = {
        .state = &power_communication_state,
    },
    .init = PowerCommunication_Init,
    .process_can_frame = PowerCommunication_ProcessCanFrame,
    .service = PowerCommunication_Service,
    .get_snapshot = PowerCommunication_GetSnapshot,
};
