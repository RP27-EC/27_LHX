#ifndef CLOUD_TERRACE_H
#define CLOUD_TERRACE_H

#include <stdbool.h>
#include <stdint.h>
#include "remote_state.h"

#define CLOUD_RC_MAX_VALUE        660.0f
#define CLOUD_MOTOR_TORQUE_MAX_NM 10.0f

// 云台归中状态。
typedef enum
{
    CLOUD_TERRACE_HOME_WAIT = 0, // 等待进入允许归中的控制模式。
    CLOUD_TERRACE_HOME_MOVING, // 两轴正在向机械中位运动。
    CLOUD_TERRACE_HOME_DONE // 归中稳定完成，允许常规控制。
} CloudTerrace_HomeState_t;

extern volatile CloudTerrace_HomeState_t cloud_terrace_home_state;
extern volatile bool cloud_turnaround_active; // 正在执行 Yaw 调头，C1 通知底盘停车。
extern volatile bool cloud_front_reversed; // 当前逻辑正方向是否为物理车尾。

// Yaw 串级控制状态。
extern volatile bool cloud_yaw_imu_online; // 上板 IMU 在线状态。
extern volatile float cloud_yaw_target_deg; // Yaw 累计目标角，单位度。
extern volatile float cloud_yaw_angle_deg; // Yaw 累计实际角，单位度。
extern volatile float cloud_yaw_rate_deg_s; // Yaw 实际角速度，单位度每秒。
extern volatile float cloud_yaw_rate_target_deg_s; // Yaw 角速度环目标值。
extern volatile int16_t cloud_yaw_torque_raw; // Yaw 原始转矩输出。

void CloudTerrace_Init(void);
void CloudTerrace_Update(const RemoteState_t *remote_snapshot);
bool CloudTerrace_LiftYawAligned(void); // 归中完成且 Yaw 位于开机机械 0° 死区内。
bool CloudTerrace_LiftPitchNonnegative(void); // Pitch 相对归中点不低于 0°。

#endif // CLOUD_TERRACE_H
