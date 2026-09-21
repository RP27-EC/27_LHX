#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* 两板通信使用 FDCAN2、11 位标准 ID、经典 CAN、8 字节数据帧。 */
#define COMMUNICATION_FRAME_SIZE       8U

/* 下板发送给上板的报文 ID。 */
#define COMMUNICATION_TX_ID_D1         0x0D1U
#define COMMUNICATION_TX_ID_D2         0x0D2U
#define COMMUNICATION_TX_ID_D3         0x0D3U
#define COMMUNICATION_TX_ID_D4         0x0D4U

/* 上板发送给下板的报文 ID，也是下板滤波器唯一放行的两个 ID。 */
#define COMMUNICATION_RX_ID_C1         0x0C1U
#define COMMUNICATION_RX_ID_C2         0x0C2U

typedef struct
{
    uint8_t data[COMMUNICATION_FRAME_SIZE]; /* 最近一帧的原始 8 字节数据。 */
    uint32_t last_rx_ms;                    /* 最近一次接收时的 HAL 毫秒时间。 */
    uint32_t rx_count;                      /* 该 ID 累计收到的有效帧数。 */
    bool received;                          /* 上电后是否至少收到过一帧。 */
} Communication_RxFrame;

/* 便于在 Keil Debug 中直接观察板间通信是否工作。 */
extern volatile uint32_t communication_rx_count;
extern volatile uint32_t communication_last_rx_id;

/* 在 MX_FDCAN2_Init() 之后调用：配置 C1/C2 滤波器、启动 FDCAN2 并开启 FIFO0 中断。 */
HAL_StatusTypeDef Communication_Init(void);

/* 发送一帧到上板。std_id 只允许 D1~D4，data 必须指向 8 字节数据。 */
HAL_StatusTypeDef Communication_Send(uint32_t std_id,
                                     const uint8_t data[COMMUNICATION_FRAME_SIZE]);

/* 原子复制 C1 或 C2 的最近接收结果；尚未收到或 ID 非法时返回 false。 */
bool Communication_GetRxFrame(uint32_t std_id, Communication_RxFrame *frame);

/* 跟随协议：C1 为归中 Yaw 角度，D4 为 BMI088 实测底盘角速度；
 * [0:1] 有符号小端 0.01 度(每秒)，[2] bit0 表示数据有效。 */
bool Communication_GetYawAngle(float *angle_deg);
HAL_StatusTypeDef Communication_SendChassisYawRate(float rate_deg_s);

/* 由统一 HAL FDCAN FIFO0 回调调用，不应由任务代码直接调用。 */
void Communication_FDCANRxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t interrupts);

/* 每收到一帧有效的 C1/C2 后在中断中调用。
 * 当前文件提供空的弱实现；后续解析协议时，可在其他 .c 文件中实现同名函数。
 * 回调运行在中断上下文，不能阻塞，也不要调用 osDelay 等 RTOS 延时函数。
 */
void Communication_RxFrameCallback(uint32_t std_id,
                                   const uint8_t data[COMMUNICATION_FRAME_SIZE]);

#endif
