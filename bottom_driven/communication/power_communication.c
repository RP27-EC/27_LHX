#include "power_communication.h"
#include "chassis_can.h"
#include "peripheral_config.h"
#include "chassis_power.h"

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
    uint32_t i;
    bool output_allowed;
    uint16_t limit = ChassisPower_GetLimit(now, &output_allowed);

    // 与模板 0x222 的 packed 结构一致，不依赖编译器结构体对齐。
    data[0] = power_communication_config.chassis_power_buffer;
    write_u16_le(&data[1], limit);
    write_u16_le(&data[3], (uint16_t)power_communication_config.cap_power_out_limit);
    // 模板在预充模式下把充电功率字段清零。
    write_u16_le(&data[5], power_communication_config.precharge_enabled ?
                 0U : power_communication_config.cap_power_in_limit);
    if (power_communication_config.cap_enabled && output_allowed) { data[7] |= 0x01U; }
    if (power_communication_config.turbo_enabled) { data[7] |= 0x02U; }
    if (power_communication_config.precharge_enabled) { data[7] |= 0x04U; }

    if (ChassisCan_Send(POWER_CAPACITOR_CONTROL_ID, data) == HAL_OK)
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
