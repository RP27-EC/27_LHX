#ifndef MOTOR4310_H
#define MOTOR4310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

/* 完整上板工程 Yaw 轴 DM4310：CAN2 发送 0x002，反馈 0x012。 */
#define MOTOR4310_CONTROL_CAN_ID         0x002U
#define MOTOR4310_FEEDBACK_CAN_ID        0x012U
#define MOTOR4310_CAN_FRAME_SIZE         8U
#define MOTOR4310_VMAX                   30.0f
#define MOTOR4310_TMAX                   2.0f
#define MOTOR4310_PMAX                   3.14159265359f
#define MOTOR4310_ECD_PER_ROUND          \
    (6.28318530718f * 65535.0f / (2.0f * MOTOR4310_PMAX))

/* 保留模板工程的两组 PID 参数与 10 ms 控制周期。 */
#define MOTOR4310_SPEED_KP               1.0f
#define MOTOR4310_SPEED_KI               0.5f
#define MOTOR4310_SPEED_KD               0.0f
#define MOTOR4310_SPEED_INTEGRAL_LIMIT   200.0f
#define MOTOR4310_SPEED_OUTPUT_LIMIT     2047.0f
#define MOTOR4310_POSITION_KP            0.5f
#define MOTOR4310_POSITION_KI            0.0f
#define MOTOR4310_POSITION_KD            0.0f
#define MOTOR4310_POSITION_INTEGRAL_LIMIT 100.0f
#define MOTOR4310_POSITION_OUTPUT_LIMIT  200.0f
#define MOTOR4310_CONTROL_PERIOD_S       0.01f

typedef struct
{
    uint16_t angle;
    int16_t speed;
    int16_t torque;
    uint8_t temperature;
    uint8_t state;
    int32_t total_angle;
    uint16_t zero_angle;
    uint16_t last_angle;
    float angle_degree;
    volatile bool initialized;
    volatile uint32_t rx_count;
    volatile uint32_t last_rx_ms;
} Motor4310_Data_t;

extern Motor4310_Data_t motor4310;
extern PID_Controller_t motor4310_speed_pid;
extern PID_Controller_t motor4310_position_pid;

/* 配置 CAN2 精确放行 0x012、启动 FIFO0 中断并初始化 PID。 */
HAL_StatusTypeDef Motor4310_Init(void);

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
