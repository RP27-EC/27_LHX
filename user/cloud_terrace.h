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
extern volatile float cloud_yaw_mechanical_error_deg; // 机械目标减反馈，deg。
extern volatile float cloud_yaw_mechanical_speed_limit_raw; // 当前回正目标速度限幅，原始码。
extern volatile bool cloud_yaw_mechanical_gyro_online; // 机械速度环 IMU 有效状态。
extern volatile float cloud_turn_progress; // S 曲线进度 0~1。
extern volatile float cloud_turn_duration_s; // 本次轨迹时长。
extern volatile float cloud_turn_target_deg; // 相对机械零点的轨迹目标角。

// Pitch IMU 惯性控制状态，角度和角速度均已换算到电机正方向。
extern volatile bool cloud_pitch_imu_online;
extern volatile float cloud_pitch_target_deg;
extern volatile float cloud_pitch_angle_deg;
extern volatile float cloud_pitch_rate_deg_s;
extern volatile float cloud_pitch_rate_target_deg_s;
extern volatile int16_t cloud_pitch_torque_raw;

void CloudTerrace_Init(void);
void CloudTerrace_Update(const RemoteState_t *remote_snapshot);
bool CloudTerrace_LiftYawAligned(void); // 归中完成且 Yaw 位于开机机械 0° 死区内。
bool CloudTerrace_LiftPitchNonnegative(void); // Pitch 相对归中点不低于 0°。

#endif // CLOUD_TERRACE_H
