#ifndef PARAMETER_H
#define PARAMETER_H

/* DM4310 CAN 地址与掉线保护；Pitch 接 CAN1，Yaw 接 CAN2。 */
#define MOTOR4310_PITCH_CONTROL_CAN_ID      0x001U
#define MOTOR4310_PITCH_FEEDBACK_CAN_ID     0x011U
#define MOTOR4310_CONTROL_CAN_ID            0x002U
#define MOTOR4310_FEEDBACK_CAN_ID           0x012U
#define MOTOR4310_OFFLINE_TIMEOUT_MS        100U
#define MOTOR4310_DISABLE_RETRY_MS          50U
/* 板间遥控帧超时；下板断遥控后停止发送 D1～D3。 */
#define COMM_RC_TIMEOUT_MS                  100U

/* 电机串级 PID，按 10 ms 控制周期整定。 */
#define MOTOR4310_SPEED_KP                  1.0f
#define MOTOR4310_SPEED_KI                  0.5f
#define MOTOR4310_SPEED_KD                  0.0f
#define MOTOR4310_SPEED_INTEGRAL_LIMIT      200.0f
#define MOTOR4310_SPEED_OUTPUT_LIMIT        2047.0f
#define MOTOR4310_POSITION_KP               0.5f
#define MOTOR4310_POSITION_KI               0.5f
#define MOTOR4310_POSITION_KD               0.0f
#define MOTOR4310_POSITION_INTEGRAL_LIMIT   100.0f
#define MOTOR4310_POSITION_OUTPUT_LIMIT     300.0f
#define MOTOR4310_CONTROL_PERIOD_S          0.01f

/* 云台任务与遥控：摇杆值范围 -660～660，速度为电机反馈原始速度码。 */
#define CLOUD_CONTROL_PERIOD_TICKS          10U
#define CLOUD_TASK_STACK_BYTES              1024U
#define CLOUD_RC_MAX_VALUE                  660.0f
#define CLOUD_RC_SPEED_ENTER                15
#define CLOUD_RC_SPEED_EXIT                 8
#define CLOUD_YAW_MAX_SPEED_RAW             300
#define CLOUD_PITCH_MAX_SPEED_RAW           150

/* 模板机械归中点；当前实车 Pitch 下限与补偿比例保留现有调试值。 */
#define CLOUD_YAW_HOME_RAD                  (-0.387884378f)
#define CLOUD_PITCH_HOME_RAD                2.59309077f
#define CLOUD_HOME_TOLERANCE_DEG            2.0f
#define CLOUD_HOME_SPEED_RAW_MAX            20
#define CLOUD_HOME_STABLE_CYCLES            20U
#define CLOUD_PITCH_MIN_DEG                 (-7.0f)
#define CLOUD_PITCH_MAX_DEG                 30.0f
#define CLOUD_PITCH_LIMIT_SLOW_DEG          5.0f
#define CLOUD_PITCH_GRAVITY_CENTER_RAD      2.678706762f
#define CLOUD_PITCH_GRAVITY_K               4.2072f
#define CLOUD_PITCH_GRAVITY_B               (-2.496f)
#define CLOUD_PITCH_GRAVITY_SCALE           0.6f
#define CLOUD_MOTOR_TORQUE_MAX_NM           10.0f

#endif /* PARAMETER_H */
