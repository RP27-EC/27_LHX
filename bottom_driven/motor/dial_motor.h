#ifndef DIAL_MOTOR_H
#define DIAL_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "parameter.h"
#include <stdbool.h>
#include <stdint.h>

#define DIAL_MOTOR_FRAME_SIZE  8U
#define DIAL_MOTOR_ENCODER_COUNTS_PER_REV  65536.0f

typedef enum
{
    DIAL_MOTOR_CMD_CLOSE = 0x80U,  /* 关闭电机命令。 */
    DIAL_MOTOR_CMD_STOP = 0x81U,   /* 停止电机命令。 */
    DIAL_MOTOR_CMD_RUN = 0x88U,    /* 运行电机命令。 */
    DIAL_MOTOR_CMD_TORQUE = 0xA1U  /* 转矩电流闭环命令。 */
} DialMotor_Command_t;

/* LK4005 基础反馈；速度单位 1 deg/s，累计角度以首帧为零点。 */
typedef struct
{
    uint8_t response_command;     /* 最近反馈帧对应的命令字。 */
    int8_t temperature;           /* 电机温度，单位摄氏度。 */
    int16_t current_raw;          /* 反馈的转矩电流原始值。 */
    int16_t speed_dps;            /* 电机轴速度，单位度每秒。 */
    uint16_t encoder;             /* 当前单圈编码器值。 */
    uint16_t last_encoder;        /* 上一帧编码器值，用于跨圈累计。 */
    int64_t encoder_total;        /* 相对上电首帧的累计编码器计数。 */
    float position_deg;           /* 累计计数换算后的电机轴角度。 */
    volatile bool initialized;    /* 是否已经用首帧建立累计零点。 */
    volatile bool received;       /* 上电后是否至少收到过一帧反馈。 */
    volatile bool online;         /* 心跳检测得到的当前在线状态。 */
    volatile uint32_t last_rx_ms; /* 最近一次反馈的毫秒时间戳。 */
    volatile uint32_t rx_count;   /* 累计接收的有效反馈帧数。 */
} DialMotor_Feedback_t;

extern DialMotor_Feedback_t dial_motor_feedback;
extern PID_Controller_t dial_motor_position_pid;
extern PID_Controller_t dial_motor_speed_pid;
extern PID_Controller_t dial_motor_continuous_speed_pid; /* 连发独立速度环。 */

/* 配置 CAN1 的 0x141 精确过滤器。 */
HAL_StatusTypeDef DialMotor_Init(void);

/* 基础状态命令。 */
HAL_StatusTypeDef DialMotor_Run(void);
HAL_StatusTypeDef DialMotor_Stop(void);
HAL_StatusTypeDef DialMotor_Close(void);

/* 0xA1 转矩电流闭环，指令会按 DIAL_MOTOR_CURRENT_LIMIT 限幅。 */
HAL_StatusTypeDef DialMotor_SetTorqueCurrent(int16_t current);

/* 累计编码器串级位控：位置外环 -> 速度内环 -> 0xA1 电流指令。
 * target_encoder_total 以上电首帧为零点，65536 计数为电机一圈；
 * 函数需每 1 ms 调用，目标单位为累计编码器计数。
 */
HAL_StatusTypeDef DialMotor_PositionControl(int64_t target_encoder_total);
/* 拨盘速度闭环；目标和反馈均为电机轴度每秒，固定 1 ms 调用。 */
HAL_StatusTypeDef DialMotor_SpeedControl(float target_speed_dps);
void DialMotor_ResetControl(void);

bool DialMotor_GetFeedback(DialMotor_Feedback_t *feedback);
bool DialMotor_OnlineCheck(void);
void DialMotor_Heartbeat(void);

/* 由工程统一 HAL CAN 回调分发，本函数不主动读 FIFO。 */
void DialMotor_ProcessCanFrame(
    CAN_HandleTypeDef *hcan,
    uint32_t std_id,
    const uint8_t data[DIAL_MOTOR_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* DIAL_MOTOR_H */
