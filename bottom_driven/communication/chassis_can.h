#ifndef DOWN_CHASSIS_CAN_H
#define DOWN_CHASSIS_CAN_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define CHASSIS_CAN_FRAME_SIZE 8U

typedef struct
{
    uint32_t last_rx_id; // 最近接收的标准 ID。
    uint32_t rx_count; // 有效帧累计数。
    uint32_t rx_error_count; // 读取失败或帧格式错误次数。
    uint32_t last_tx_id; // 最近成功入队的标准 ID。
    uint32_t tx_count; // 成功入队帧数。
    uint32_t tx_error_count; // 发送入队失败次数。
} ChassisCanState;

extern volatile ChassisCanState chassis_can_state;

// 统一配置电机、电容和无线充滤波器，启动 FDCAN1 接收。
HAL_StatusTypeDef ChassisCan_Init(void);
// FDCAN1 标准 ID、经典 CAN、8 字节数据帧的统一发送入口。
HAL_StatusTypeDef ChassisCan_Send(uint32_t std_id,
                                 const uint8_t data[CHASSIS_CAN_FRAME_SIZE]);
// 由统一 HAL FIFO0 回调调用。
void ChassisCan_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t interrupts);

#endif
