#ifndef MOTOR3508_H
#define MOTOR3508_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define MOTOR3508_COUNT          4U
#define MOTOR3508_FEEDBACK_BASE  0x201U
#define MOTOR3508_COMMAND_ID     0x200U
#define MOTOR3508_CURRENT_LIMIT  16384

/* 反馈值均来自 C620 原始报文，不包含 PID 或底盘解算。 */
typedef struct
{
    uint16_t encoder;       /* 电机转子单圈编码器：0~8191。 */
    int16_t speed_rpm;      /* 转子转速 rpm，尚未除减速比。 */
    int16_t current_raw;    /* 有符号反馈电流原始值。 */
    uint8_t temperature;   /* 温度，摄氏度。 */
    bool received;         /* 是否至少接收过一帧；不代表当前在线。 */
    uint32_t last_rx_ms;    /* 最近一次反馈的 HAL 毫秒时间。 */
    uint32_t rx_count;      /* 该电机接收帧数。 */
} Motor3508_Feedback;

/* 在 MX_FDCAN1_Init 后调用；配置过滤器、启动总线和接收中断。 */
HAL_StatusTypeDef Motor3508_Init(void);

/* 一帧同时设置 ID 1~4 的电流指令，自动限制到 -16384~16384。
 * HAL_OK 只表示成功入队；调用方应周期发送并检查返回值。
 * 正负方向由电机安装和接线决定，不自动修正轮子方向。
 */
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t id1, int16_t id2,
                                      int16_t id3, int16_t id4);

/* 发送一帧四电机零电流指令，不是机械刹车或周期发送任务。 */
HAL_StatusTypeDef Motor3508_Stop(void);

/* motor_id 为 1~4；原子复制反馈，未收到反馈或参数错误返回 false。 */
bool Motor3508_GetFeedback(uint8_t motor_id, Motor3508_Feedback *feedback);

#endif
