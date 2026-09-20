#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 下板发送给上板的标准帧 ID。 */
#define COMM_CAN_RX_ID_D1  0x0D1U
#define COMM_CAN_RX_ID_D2  0x0D2U
#define COMM_CAN_RX_ID_D3  0x0D3U
#define COMM_CAN_RX_ID_D4  0x0D4U

/* 上板发送给下板的标准帧 ID。 */
#define COMM_CAN_TX_ID_C1  0x0C1U
#define COMM_CAN_TX_ID_C2  0x0C2U

#define COMM_CAN_FRAME_SIZE  8U

/* 每个接收 ID 保存一份最新快照，便于任务读取和调试器观察。 */
typedef struct
{
    uint8_t data[COMM_CAN_FRAME_SIZE];
    uint32_t rx_count;
    uint32_t last_rx_ms;
    bool received;
} Communication_CanRxFrame_t;

/* 在 MX_CAN2_Init() 之后调用：配置精确 ID 过滤器、启动 CAN2 和接收中断。 */
HAL_StatusTypeDef Communication_CAN_Init(void);

/* 发送固定 8 字节标准帧；std_id 只允许 0xC1 或 0xC2。 */
HAL_StatusTypeDef Communication_CAN_Send(uint16_t std_id,
                                         const uint8_t data[COMM_CAN_FRAME_SIZE]);
HAL_StatusTypeDef Communication_CAN_SendC1(
    const uint8_t data[COMM_CAN_FRAME_SIZE]);
HAL_StatusTypeDef Communication_CAN_SendC2(
    const uint8_t data[COMM_CAN_FRAME_SIZE]);

/* 获取指定 D1~D4 的最新完整快照；尚未收到或参数错误时返回 false。 */
bool Communication_CAN_GetLatest(uint16_t std_id,
                                 Communication_CanRxFrame_t *frame);

/* 收到有效 D1~D4 后调用。业务层可提供强定义覆盖此弱回调。 */
void Communication_CAN_OnReceive(uint16_t std_id,
                                 const uint8_t data[COMM_CAN_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
