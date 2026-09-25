#ifndef MOTOR2006_H
#define MOTOR2006_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

/* 模板上板升降电机：CAN1 的 0x204 回传，0x200 群组帧第 4 槽发送。 */
#define MOTOR2006_FEEDBACK_CAN_ID  0x204U
#define MOTOR2006_COMMAND_CAN_ID   0x200U
#define MOTOR2006_FRAME_SIZE       8U

typedef struct
{
    uint16_t encoder;         /* 电机转子单圈编码器，0~8191。 */
    uint16_t last_encoder;    /* 上一帧编码器，用于跨圈累计。 */
    int32_t encoder_total;    /* 以上电首帧为零点的转子累计计数。 */
    int16_t speed_rpm;        /* 电机转子速度，rpm。 */
    int16_t current_raw;      /* C610 回传电流原始码。 */
    uint8_t temperature;      /* 电机温度，摄氏度。 */
    volatile bool received;   /* 是否收到过有效反馈。 */
    volatile bool online;     /* 最近 100 ms 内是否收到反馈。 */
    volatile uint32_t last_rx_ms; /* 最近反馈时间。 */
    volatile uint32_t rx_count;   /* 有效反馈累计帧数。 */
} Motor2006_Feedback_t;

extern Motor2006_Feedback_t motor2006_feedback;
extern PID_Controller_t motor2006_speed_pid;

/* 配置 CAN1 精确过滤器；不会主动给电机非零电流。 */
HAL_StatusTypeDef Motor2006_Init(void);
/* 原始电流码经 ±10000 限幅；与摩擦轮共用 0x200 帧。
 * 非零命令须持续刷新，超过 MOTOR2006_COMMAND_TIMEOUT_MS 自动归零。 */
HAL_StatusTypeDef Motor2006_SetCurrent(int16_t current_raw);
HAL_StatusTypeDef Motor2006_Stop(void);
/* 模板升降速度环：目标为转子 rad/s，调用周期 1 ms。 */
HAL_StatusTypeDef Motor2006_SpeedControl(float target_rotor_rad_s);
void Motor2006_ResetSpeedPID(void);

bool Motor2006_GetFeedback(Motor2006_Feedback_t *feedback);
bool Motor2006_OnlineCheck(void);
void Motor2006_Heartbeat(void);
float Motor2006_GetOutputAngleDeg(void); /* 36:1 减速箱输出轴相对上电角度。 */

/* 0x200 群组帧由本驱动统一拼接：第 1/2 槽摩擦轮，第 3 槽零，第 4 槽 2006。 */
HAL_StatusTypeDef Motor2006_SendFrictionCurrents(int16_t left_raw,
                                                  int16_t right_raw);
/* 由统一 CAN 接收回调分发，不主动读取 FIFO。 */
void Motor2006_ProcessCanFrame(CAN_HandleTypeDef *hcan,
                               uint32_t std_id,
                               const uint8_t data[MOTOR2006_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR2006_H */
