#ifndef CLOUD_TERRACE_H
#define CLOUD_TERRACE_H

#include <stdbool.h>
#include <stdint.h>

/* 云台状态仅供调试观察；控制入口由固定周期任务调用。 */
typedef enum
{
    CLOUD_TERRACE_HOME_WAIT = 0,
    CLOUD_TERRACE_HOME_MOVING,
    CLOUD_TERRACE_HOME_DONE
} CloudTerrace_HomeState_t;

extern volatile CloudTerrace_HomeState_t cloud_terrace_home_state;

/* Keil Watch：上板 IMU 与 Yaw 角度-角速度串级环。 */
extern volatile bool cloud_yaw_imu_online;
extern volatile float cloud_yaw_target_deg;
extern volatile float cloud_yaw_angle_deg;
extern volatile float cloud_yaw_rate_deg_s;
extern volatile float cloud_yaw_rate_target_deg_s;
extern volatile int16_t cloud_yaw_torque_raw;

void CloudTerrace_Init(void);
void CloudTerrace_Update(void);

#endif /* CLOUD_TERRACE_H */
