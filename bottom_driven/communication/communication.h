#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "parameter.h"
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

/* 下板传来的 DJI DBUS 原始遥控帧长度。 */
#define COMM_RC_FRAME_SIZE   18U

/* 三档拨杆原始编码：上 1，中 3，下 2。 */
#define COMM_RC_SW_UP        1U
#define COMM_RC_SW_MID       3U
#define COMM_RC_SW_DOWN      2U

/* 每个接收 ID 保存一份最新快照，便于任务读取和调试器观察。 */
typedef struct
{
    uint8_t data[COMM_CAN_FRAME_SIZE];
    uint32_t rx_count;
    uint32_t last_rx_ms;
    bool received;
} Communication_CanRxFrame_t;

/* 与下板遥控器解析结果保持一致，通道值范围通常为 -660~660。 */
typedef struct
{
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;
} Communication_RcControl_t;

/* 便于 Keil Debug 直接观察；任务代码优先使用 Communication_RC_Get。 */
extern Communication_RcControl_t communication_rc;
extern volatile bool communication_rc_online;
extern volatile uint32_t communication_rc_valid_count;
extern volatile uint32_t communication_rc_last_valid_ms;
extern volatile uint32_t communication_rc_assembly_error_count;

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

/*
 * 由通信任务周期调用：处理 ISR 提交的完整 18 字节帧并执行断信检测。
 * 建议周期 1~10 ms；断信超过 100 ms 后自动清零摇杆并将拨杆置中。
 */
void Communication_Process(void);

/* 解析一帧完整 DBUS 数据；公开此接口便于脱离 CAN 做单元调试。 */
bool Communication_RC_Parse(
    const uint8_t frame[COMM_RC_FRAME_SIZE],
    Communication_RcControl_t *control);

/* 原子复制当前遥控结果；返回值表示遥控链路当前是否在线。 */
bool Communication_RC_Get(Communication_RcControl_t *control);
bool Communication_RC_IsOnline(void);

/* 收到有效 D1~D4 后调用。业务层可提供强定义覆盖此弱回调。 */
void Communication_CAN_OnReceive(uint16_t std_id,
                                 const uint8_t data[COMM_CAN_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATION_H */
