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
    DIAL_MOTOR_CMD_CLOSE = 0x80U,
    DIAL_MOTOR_CMD_STOP = 0x81U,
    DIAL_MOTOR_CMD_RUN = 0x88U,
    DIAL_MOTOR_CMD_TORQUE = 0xA1U
} DialMotor_Command_t;

/* LK4005 基础反馈；速度单位 1 deg/s，累计角度以首帧为零点。 */
typedef struct
{
    uint8_t response_command;
    int8_t temperature;
    int16_t current_raw;
    int16_t speed_dps;
    uint16_t encoder;
    uint16_t last_encoder;
    int64_t encoder_total;
    float position_deg;
    volatile bool initialized;
    volatile bool received;
    volatile bool online;
    volatile uint32_t last_rx_ms;
    volatile uint32_t rx_count;
} DialMotor_Feedback_t;

extern DialMotor_Feedback_t dial_motor_feedback;
extern PID_Controller_t dial_motor_position_pid;
extern PID_Controller_t dial_motor_speed_pid;

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
 * 函数需每 1 ms 调用，与模板拨盘的 target_anglesum 单位一致。
 */
HAL_StatusTypeDef DialMotor_PositionControl(int64_t target_encoder_total);
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
