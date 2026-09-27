#ifndef DOWN_POWER_COMMUNICATION_H
#define DOWN_POWER_COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define POWER_CAPACITOR_STATUS_ID 0x211U
#define POWER_WIRELESS_STATUS_ID  0x212U
#define POWER_CAPACITOR_CONTROL_ID 0x222U

// 电容状态：电压、电流
typedef struct
{
    int16_t chassis_power_raw; // 字节 0~1，设备上报的底盘功率原值。
    int16_t voltage_raw; // 字节 2~3，电容电压原值。
    int16_t current_raw; // 字节 4~5，电容电流原值。
    float voltage_v; // 电压换算值，V。
    float current_a; // 电流换算值，A。
    bool discharge_available; // 字节 6 的 bit0，允许放电。
    bool precharge_active; // 字节 6 的 bit1，正在预充。
    uint8_t raw[8]; // 最近一帧原始数据。
    uint32_t last_rx_ms; // 最近一次接收时间，ms。
    uint32_t rx_count; // 累计接收帧数。
    bool received; // 是否曾收到有效帧。
    bool online; // 最近一帧是否仍在超时范围内。
} CapacitorStatus;

// 无线充状态
typedef struct
{
    int16_t charging_power_raw; // 字节 0~1，充电功率原值。
    float charging_power_w; // 功率换算值，W。
    bool charging; // 字节 2，非零表示正在充电。
    uint8_t raw[8]; // 最近一帧原始数据。
    uint32_t last_rx_ms; // 最近一次接收时间，ms。
    uint32_t rx_count; // 累计接收帧数。
    bool received; // 是否曾收到有效帧。
    bool online; // 最近一帧是否仍在超时范围内。
} WirelessChargeStatus;

typedef struct
{
    CapacitorStatus capacitor; // 0x211 电容状态。
    WirelessChargeStatus wireless; // 0x212 无线充状态。
    struct
    {
        uint8_t raw[8]; // 最近一次成功入队的 0x222 数据。
        uint32_t last_tx_ms; // 最近一次成功入队时间，ms。
        uint32_t tx_count; // 成功入队次数。
        uint32_t tx_error_count; // 发送入队失败次数。
    } control; // 超电基础控制帧发送状态。
} PowerCommunicationState;

extern volatile PowerCommunicationState power_communication_state;

// 清空上电状态；FDCAN1 的滤波与启动由统一底盘 CAN 接口负责。
void PowerCommunication_Init(void);
// 统一底盘 CAN 接口分发已校验的电容/无线充状态帧。
void PowerCommunication_ProcessCanFrame(uint32_t id, const uint8_t data[8]);
// 定期更新在线标志，并按配置周期发送 0x222 基础控制帧。
void PowerCommunication_Service(void);
// 原子读取状态快照，供任务或调试代码使用。
bool PowerCommunication_GetSnapshot(PowerCommunicationState *snapshot);

#endif
