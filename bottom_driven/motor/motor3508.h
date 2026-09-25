#ifndef MOTOR3508_H
#define MOTOR3508_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "parameter.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR3508_COUNT          2U
#define MOTOR3508_FEEDBACK_BASE  0x201U
#define MOTOR3508_COMMAND_ID     0x200U
#define MOTOR3508_FRAME_SIZE     8U

/* C620 反馈原始数据；速度单位为电机转子 rpm。 */
typedef struct
{
    uint16_t encoder;            /* 转子单圈编码器原始值，范围 0~8191。 */
    int16_t speed_rpm;           /* 转子反馈转速，单位 rpm。 */
    int16_t current_raw;         /* C620 反馈的实际电流原始值。 */
    uint8_t temperature;         /* 电机温度，单位摄氏度。 */
    volatile bool received;      /* 上电后是否至少收到过一帧反馈。 */
    volatile bool online;        /* 心跳检测得到的当前在线状态。 */
    volatile uint32_t last_rx_ms;/* 最近一次反馈的毫秒时间戳。 */
    volatile uint32_t rx_count;  /* 累计接收的有效反馈帧数。 */
} Motor3508_Feedback_t;

extern Motor3508_Feedback_t motor3508_feedback[MOTOR3508_COUNT];
extern PID_Controller_t motor3508_speed_pid[MOTOR3508_COUNT];

/* 配置 CAN1 上 0x201~0x204 过滤器，并初始化四路速度 PID。 */
HAL_StatusTypeDef Motor3508_Init(void);

/* 发送 0x200 群组电流帧；ID1/ID2 为摩擦轮，ID3 为零，ID4 保留 M2006 电流。 */
HAL_StatusTypeDef Motor3508_SendCurrent(int16_t current_1,
                                       int16_t current_2);

/* 单输入双摩擦轮速度 PID：ID1 与 ID2 的目标等大反向；
 * 目标会按 MOTOR3508_MAX_SPEED_RPM 做双向限速，调用周期必须与
 * MOTOR3508_PID_CONTROL_TIME_S 一致。
 */
HAL_StatusTypeDef Motor3508_SpeedControl(int16_t target_speed_rpm);
HAL_StatusTypeDef Motor3508_Stop(void);
void Motor3508_ResetSpeedPID(void);

/* motor_id 范围为 1~2。 */
bool Motor3508_GetFeedback(uint8_t motor_id,
                           Motor3508_Feedback_t *feedback);
bool Motor3508_OnlineCheck(uint8_t motor_id);
bool Motor3508_AllOnline(void);
void Motor3508_Heartbeat(void);

/* 由工程统一 HAL CAN 回调分发，本函数不主动读 FIFO。 */
void Motor3508_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[MOTOR3508_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR3508_H */
