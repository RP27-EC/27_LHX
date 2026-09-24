#ifndef MOTOR4310_H
#define MOTOR4310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "parameter.h"
#include <stdbool.h>
#include <stdint.h>

/* MIT 协议固定量；可调参数见 user/parameter.h。 */
#define MOTOR4310_CAN_FRAME_SIZE         8U /* 经典 CAN 数据帧长度，字节。 */
#define MOTOR4310_PMAX                   3.14159265359f /* 单圈位置范围为 -π~π rad。 */
/* 位置回传 16 位，-π~π 映射到 0~65535：一圈计数=2π×65535/(2×PMAX)，约 65535。 */
#define MOTOR4310_ECD_PER_ROUND          \
    (6.28318530718f * 65535.0f / (2.0f * MOTOR4310_PMAX))

typedef enum
{
    MOTOR4310_PITCH = 0, /* Pitch 轴电机数组下标。 */
    MOTOR4310_YAW = 1,   /* Yaw 轴电机数组下标。 */
    MOTOR4310_COUNT = 2  /* 云台 4310 电机总数。 */
} Motor4310_Id_t;

typedef struct
{
    uint16_t angle;       /* MIT 反馈的单圈位置原始值。 */
    int16_t speed;        /* MIT 反馈的速度原始值。 */
    int16_t torque;       /* MIT 反馈的转矩原始值。 */
    uint8_t temperature;  /* 电机温度，单位摄氏度。 */
    uint8_t state;        /* MIT 反馈帧中的电机状态码。 */
    int32_t total_angle; /* 以编码器数值 0 为零点的累计计数。 */
    uint16_t zero_angle; /* 固定为 0；保留字段兼容现有调试观察。 */
    uint16_t last_angle;  /* 上一帧单圈位置，用于累计圈数。 */
    float angle_degree;   /* 当前累计位置换算得到的角度。 */
    volatile bool initialized;    /* 是否已用首帧建立累计位置。 */
    volatile uint32_t rx_count;   /* 累计接收的有效反馈帧数。 */
    volatile uint32_t last_rx_ms; /* 最近一次反馈的毫秒时间戳。 */
    volatile bool online;         /* 心跳检测得到的当前在线状态。 */
    volatile bool enabled; /* 最近一次成功入队的使能/失能命令状态，不等于电机硬件确认。 */
    uint32_t last_enable_ms; /* 最近一次发送使能命令的时间。 */
    volatile bool disable_command_sent; /* 仅表示上次失能命令已入队，断联时仍会重发。 */
    uint32_t last_disable_ms; /* 最近一次发送失能命令的时间。 */
} Motor4310_Data_t;

extern Motor4310_Data_t motor4310_data[MOTOR4310_COUNT];
extern PID_Controller_t motor4310_speed_pids[MOTOR4310_COUNT];
extern PID_Controller_t motor4310_position_pids[MOTOR4310_COUNT];
/* 原单 Yaw 调试名称保持可用。 */
#define motor4310 motor4310_data[MOTOR4310_YAW]
#define motor4310_speed_pid motor4310_speed_pids[MOTOR4310_YAW]
#define motor4310_position_pid motor4310_position_pids[MOTOR4310_YAW]

/* 分别配置 CAN1/2 的反馈过滤器；CAN2 与板间通信共用接收回调。 */
HAL_StatusTypeDef Motor4310_Init(void);
/* 两轴独立指定 id 控制；位置/速度闭环要求两轴同时在线。 */
bool Motor4310_OnlineCheck(Motor4310_Id_t id);
bool Motor4310_AllOnline(void);
bool Motor4310_GetFeedback(Motor4310_Id_t id, Motor4310_Data_t *feedback);
void Motor4310_Heartbeat(void);
void Motor4310_ResetControl(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_EnableMotor(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_DisableMotor(Motor4310_Id_t id);
HAL_StatusTypeDef Motor4310_PositionControlMotor(Motor4310_Id_t id,
                                                 int32_t target_position);
HAL_StatusTypeDef Motor4310_PositionControlWithFeedforward(
    Motor4310_Id_t id, int32_t target_position, int16_t feedforward_raw);
HAL_StatusTypeDef Motor4310_SpeedControlMotor(Motor4310_Id_t id,
                                              int16_t target_speed);
HAL_StatusTypeDef Motor4310_SpeedControlWithFeedforward(
    Motor4310_Id_t id, int16_t target_speed, int16_t feedforward_raw);
HAL_StatusTypeDef Motor4310_SetTorqueRawMotor(Motor4310_Id_t id,
                                              int16_t torque);
void Motor4310_ParseFeedbackMotor(Motor4310_Id_t id,
                                  const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);
void Motor4310_ProcessCanFrame(CAN_HandleTypeDef *hcan, uint32_t std_id,
                               const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);

/* MIT 模式特殊命令与纯转矩输出。 */
HAL_StatusTypeDef Motor4310_Enable(void);
HAL_StatusTypeDef Motor4310_Disable(void);
HAL_StatusTypeDef Motor4310_SetTorqueRaw(int16_t torque);

/* 速度环与位置-速度串级环。 */
HAL_StatusTypeDef Motor4310_SpeedControl(int16_t target_speed);
HAL_StatusTypeDef Motor4310_PositionControl(int32_t target_position);
int32_t Motor4310_PositionToEcd(float rounds, float degree);

void Motor4310_ParseFeedback(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);
void Motor4310_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR4310_H */
