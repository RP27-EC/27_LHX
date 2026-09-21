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

/* MIT 协议固定的计数范围；可调参数统一见 user/parameter.h。 */
#define MOTOR4310_CAN_FRAME_SIZE         8U
#define MOTOR4310_PMAX                   3.14159265359f
#define MOTOR4310_ECD_PER_ROUND          \
    (6.28318530718f * 65535.0f / (2.0f * MOTOR4310_PMAX))

typedef enum
{
    MOTOR4310_PITCH = 0,
    MOTOR4310_YAW = 1,
    MOTOR4310_COUNT = 2
} Motor4310_Id_t;

typedef struct
{
    uint16_t angle;
    int16_t speed;
    int16_t torque;
    uint8_t temperature;
    uint8_t state;
    int32_t total_angle; /* 以编码器数值 0 为零点的累计计数。 */
    uint16_t zero_angle; /* 固定为 0；保留字段兼容现有调试观察。 */
    uint16_t last_angle;
    float angle_degree;
    volatile bool initialized;
    volatile uint32_t rx_count;
    volatile uint32_t last_rx_ms;
    volatile bool online;
    volatile bool enabled; /* 最近一次成功入队的使能/失能命令状态，不等于电机硬件确认。 */
    uint32_t last_enable_ms;
    volatile bool disable_command_sent; /* 仅表示上次失能命令已入队，断联时仍会重发。 */
    uint32_t last_disable_ms;
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

/* 模板的速度环与位置-速度串级环。 */
HAL_StatusTypeDef Motor4310_SpeedControl(int16_t target_speed);
HAL_StatusTypeDef Motor4310_PositionControl(int32_t target_position);
int32_t Motor4310_PositionToEcd(float rounds, float degree);

void Motor4310_ParseFeedback(const uint8_t data[MOTOR4310_CAN_FRAME_SIZE]);
void Motor4310_CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR4310_H */
