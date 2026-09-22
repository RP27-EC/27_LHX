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

/* 电机与云台 PID，按 1 ms 控制周期运行。 */
#define MOTOR4310_SPEED_KP                  1.1f
#define MOTOR4310_SPEED_KI                  0.5f
#define MOTOR4310_SPEED_KD                  0.0f
#define MOTOR4310_SPEED_INTEGRAL_LIMIT      200.0f
#define MOTOR4310_SPEED_OUTPUT_LIMIT        2047.0f
#define MOTOR4310_POSITION_KP               0.6f
#define MOTOR4310_POSITION_KI               0.5f
#define MOTOR4310_POSITION_KD               0.0f
#define MOTOR4310_POSITION_INTEGRAL_LIMIT   100.0f
#define MOTOR4310_POSITION_OUTPUT_LIMIT     300.0f
#define MOTOR4310_CONTROL_PERIOD_S          0.001f

/* 云台任务与遥控：摇杆值范围 -660～660。 */
#define CLOUD_CONTROL_PERIOD_TICKS          1U
#define CLOUD_TASK_STACK_BYTES              1024U
#define CLOUD_RC_MAX_VALUE                  660.0f
#define CLOUD_RC_SPEED_ENTER                15
#define CLOUD_RC_SPEED_EXIT                 8
#define CLOUD_PITCH_MAX_SPEED_RAW           150

/* D4 底盘角速度仅保留作调试数据，不再参与 Yaw 控制。 */
#define CLOUD_FOLLOW_RATE_TIMEOUT_MS        100U

/* 上板 BMI088：模板坐标变换为绕 Z 轴180°（X/Y 取反，Z 不变）。 */
#define GIMBAL_IMU_UPDATE_PERIOD_S          0.001f
#define GIMBAL_IMU_CALIBRATION_SAMPLES      1000U
#define GIMBAL_IMU_ATTITUDE_KP              2.0f
#define GIMBAL_IMU_ATTITUDE_KI              0.02f
#define GIMBAL_IMU_YAW_RATE_FILTER_ALPHA    1.0f

/*  Yaw 惯性系位控：角度外环 -> 陀螺仪角速度内环 -> 转矩。 */
#define CLOUD_YAW_COMMAND_RATE_DEG_S        200.0f
#define CLOUD_YAW_RC_DIRECTION              1.0f
#define CLOUD_YAW_ANGLE_KP                  22.0f
#define CLOUD_YAW_ANGLE_KI                  0.1f
#define CLOUD_YAW_ANGLE_KD                  0.0f
#define CLOUD_YAW_ANGLE_INTEGRAL_LIMIT      200.0f
#define CLOUD_YAW_RATE_TARGET_LIMIT_DEG_S   500.0f
/* 模板内环 Kp=0.04 Nm/(deg/s)，4310 原始转矩码按 2047/10 Nm 换算。 */
#define CLOUD_YAW_RATE_KP                   9.0f
#define CLOUD_YAW_RATE_KI                   0.1f
#define CLOUD_YAW_RATE_KD                   0.0f
#define CLOUD_YAW_RATE_INTEGRAL_LIMIT       0.0f
#define CLOUD_YAW_TORQUE_LIMIT_RAW          2047.0f

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
