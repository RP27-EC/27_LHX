#ifndef CLOUD_TERRACE_H
#define CLOUD_TERRACE_H

#include <stdbool.h>
#include <stdint.h>

/* 云台状态仅供调试观察；控制入口由固定周期任务调用。 */
typedef enum
{
    CLOUD_TERRACE_HOME_WAIT = 0, /* 等待进入允许归中的控制模式。 */
    CLOUD_TERRACE_HOME_MOVING,  /* 两轴正在向机械中位运动。 */
    CLOUD_TERRACE_HOME_DONE     /* 归中稳定完成，允许常规控制。 */
} CloudTerrace_HomeState_t;

extern volatile CloudTerrace_HomeState_t cloud_terrace_home_state;

/* Keil Watch：上板 IMU 与 Yaw 角度-角速度串级环。 */
extern volatile bool cloud_yaw_imu_online;         /* 上板 IMU 在线状态。 */
extern volatile float cloud_yaw_target_deg;        /* Yaw 累计目标角，单位度。 */
extern volatile float cloud_yaw_angle_deg;         /* Yaw 累计实际角，单位度。 */
extern volatile float cloud_yaw_rate_deg_s;        /* Yaw 实际角速度，单位度每秒。 */
extern volatile float cloud_yaw_rate_target_deg_s; /* Yaw 角速度环目标值。 */
extern volatile int16_t cloud_yaw_torque_raw;      /* Yaw 原始转矩输出。 */

void CloudTerrace_Init(void);
void CloudTerrace_Update(void);

#endif /* CLOUD_TERRACE_H */
