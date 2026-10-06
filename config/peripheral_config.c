#include "peripheral_config.h"
#include <stddef.h>

volatile Motor4310Config motor4310_config; // 云台电机闭环与通信参数。

// 加载 Yaw 机械保持用的电机位置、速度串级 PID 默认值。
void Motor4310YawHoldConfig_Init(volatile Motor4310_PidProfile_t *config)
{
    if (config == NULL) { return; }
    config->speed.kp = 1.4f; // Yaw 机械保持速度环比例增益。
    config->speed.ki = 0.2f; // 4310 速度环积分增益。
    config->speed.kd = 0.0f; // 4310 速度环微分增益。
    config->speed.integral_limit = 350.0f; // 速度环积分项限幅。
    config->speed.output_limit = 2047.0f; // 速度环转矩码输出限幅。
    config->position.kp = 0.4f; // Yaw 机械保持位置环比例增益。
    config->position.ki = 0.15f; // 4310 位置环积分增益。
    config->position.kd = 0.001f; // 4310 位置环微分增益。
    config->position.integral_limit = 150.0f; // 位置环积分项限幅。
    config->position.output_limit = 400.0f; // 位置环目标速度限幅。
}

// 加载 Yaw 调头专用的电机位置、速度串级 PID 默认值。
void Motor4310YawTurnConfig_Init(volatile Motor4310_PidProfile_t *config)
{
    if (config == NULL) { return; }
    config->position.kp = 0.8f; // 调头独立位置环比例增益。
    config->position.ki = 0.0f; // 调头独立位置环积分增益。
    config->position.kd = 0.0001f; // 调头独立位置环微分增益。
    config->position.integral_limit = 0.0f; // 调头独立位置环积分项限幅。
    config->position.output_limit = 1500.0f; // 调头独立位置环目标速度限幅，原始码。
    config->speed.kp = 1.8f; // 调头独立速度环比例增益。
    config->speed.ki = 0.1f; // 调头独立速度环积分增益。
    config->speed.kd = 0.0f; // 调头独立速度环微分增益。
    config->speed.integral_limit = 0.0f; // 调头独立速度环积分项限幅。
    config->speed.output_limit = 2047.0f; // 调头独立速度环转矩输出限幅，原始码。
}

// 加载 Pitch 电机位置、速度串级 PID 默认值。
void Motor4310PitchConfig_Init(volatile Motor4310_PidProfile_t *config)
{
    if (config == NULL) { return; }
    config->position.kp = 0.35f; // Pitch 位置环比例增益。
    config->position.ki = 0.1f; // Pitch 位置环积分增益。
    config->position.kd = 0.004f; // Pitch 位置环微分增益。
    config->position.integral_limit = 100.0f; // Pitch 位置环积分限幅。
    config->position.output_limit = 600.0f; // Pitch 目标速度限幅。
    config->speed.kp = 1.2f; // Pitch 速度环比例增益。
    config->speed.ki = 0.0f; // Pitch 速度环积分增益。
    config->speed.kd = 0.003f; // Pitch 速度环微分增益。
    config->speed.integral_limit = 200.0f; // Pitch 速度环积分限幅。
    config->speed.output_limit = 2047.0f; // Pitch 转矩码限幅。
}

// 加载云台电机通信保护、控制周期、固定前馈及各轴 PID 默认值。
void Motor4310Config_Init(void)
{
    motor4310_config.offline_timeout_ms = 100U; // 电机反馈离线超时，ms。
    motor4310_config.disable_retry_ms = 50U; // 失能命令重发间隔，ms。
    motor4310_config.control_period_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
    motor4310_config.yaw_speed_feedforward_raw = 0.0f; // Yaw 转动时按目标方向叠加的固定转矩码。
    motor4310_config.yaw_speed_feedforward_deadband_raw = 0.0f; // 4310 原始目标速度超过此值才加前馈。
    Motor4310YawHoldConfig_Init(&motor4310_config.yaw_hold);
    Motor4310YawTurnConfig_Init(&motor4310_config.yaw_turn);
    Motor4310PitchConfig_Init(&motor4310_config.pitch);
}

volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。

// 加载下板遥控和底盘角速度反馈的接收有效期。
void CommunicationConfig_Init(void)
{
    communication_config.timeout_ms = 100U; // 从 D1 接收起计算遥控有效期，ms。
    communication_config.yaw_rate_timeout_ms = 100U; // 下板角速度帧有效期，ms。
}

volatile Motor3508Config motor3508_config; // 电机闭环与命令保护参数。

// 加载摩擦轮速度环 PID 和电流输出限幅。
void Motor3508SpeedPidConfig_Init(volatile Motor3508SpeedPidConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 5.0f; // 摩擦轮速度环比例增益。
    config->ki = 2.0f; // 摩擦轮速度环积分增益。
    config->kd = 0.0f; // 摩擦轮速度环微分增益。
    config->integral_limit = 500.0f; // 速度环积分项限幅。
    config->output_limit = 16384.0f; // 速度环电流码输出限幅。
}

// 加载主动停轮增益、电流上限和停止死区。
void Motor3508BrakeConfig_Init(volatile Motor3508BrakeConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 5.0f; // 轮速越高，反向制动力矩越大。
    config->current_limit_raw = 5000; // 限制刹车时的反向电流。
    config->stop_speed_rpm = 20; // 接近停止后撤掉电流，避免来回反转。
}

// 加载摩擦轮方向、转速、电流保护、在线超时及闭环默认值。
void Motor3508Config_Init(void)
{
    motor3508_config.current_limit = 16384; // 摩擦轮电流命令限幅，原始码。
    motor3508_config.max_speed_rpm = 8000.0f; // 摩擦轮目标转速上限，rpm。
    motor3508_config.left_direction = 1.0f; // 左摩擦轮转向系数。
    motor3508_config.right_direction = (-1.0f); // 右摩擦轮转向系数。
    motor3508_config.offline_timeout_ms = 100U; // 摩擦轮反馈离线超时，ms。
    motor3508_config.pid_control_time_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
    Motor3508SpeedPidConfig_Init(&motor3508_config.speed);
    Motor3508BrakeConfig_Init(&motor3508_config.brake);
}

volatile Motor2006Config motor2006_config; // 升降电机速度环与命令保护参数。

// 加载升降电机速度环 PID 和电流输出限幅。
void Motor2006SpeedPidConfig_Init(volatile Motor2006SpeedPidConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 12.0f; // 升降电机速度环比例增益。
    config->ki = 5.0f; // 升降电机速度环积分增益。
    config->kd = 0.0f; // 升降电机速度环微分增益。
    config->integral_limit = 0.0f; // 速度环积分项限幅。
    config->output_limit = 800.0f; // 速度环转矩输出限幅。
}

// 加载升降电机电流保护、命令超时、转矩换算及速度环默认值。
void Motor2006Config_Init(void)
{
    motor2006_config.current_limit = 10000; // 升降电机电流命令限幅，原始码。
    motor2006_config.offline_timeout_ms = 100U; // 升降电机反馈离线超时，ms。
    motor2006_config.command_timeout_ms = 100U; // 控制命令超时清零间隔，ms。
    motor2006_config.torque_constant = 0.18f; // 速度环转矩到电流的换算系数。
    motor2006_config.pid_control_time_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
    Motor2006SpeedPidConfig_Init(&motor2006_config.speed);
}

volatile DialMotorConfig dial_motor_config; // 拨盘电机闭环与通信参数。

// 加载拨盘位置外环 PID 和目标速度限幅。
void DialPositionPidConfig_Init(volatile DialPositionPidConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 0.09f; // 拨盘位置环比例增益。
    config->ki = 0.05f; // 拨盘位置环积分增益。
    config->kd = 0.0f; // 拨盘位置环微分增益。
    config->integral_limit = 0.0f; // 位置环积分项限幅。
    config->speed_limit_dps = 2500.0f; // 位置环目标速度限幅，度/s。
}

// 加载拨盘单发速度内环 PID 及单发、连发共用的积分与电流限幅。
void DialSpeedPidConfig_Init(volatile DialSpeedPidConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 0.07f; // 单发速度环比例增益。
    config->ki = 0.03f; // 单发速度环积分增益。
    config->kd = 0.0f; // 单发速度环微分增益。
    config->integral_limit = 500.0f; // 速度环积分项限幅。
    config->output_limit = 1600.0f; // 速度环电流码输出限幅。
}

// 加载拨盘连发专用速度环 PID 增益。
void DialContinuousSpeedPidConfig_Init(volatile DialContinuousSpeedPidConfig *config)
{
    if (config == NULL) { return; }
    config->kp = 0.12f; // 连发速度环比例增益。
    config->ki = 0.5f; // 连发速度环积分增益。
    config->kd = 0.0f; // 连发速度环微分增益。
}

// 加载拨盘电流保护、通信间隔、控制周期及位置、速度闭环默认值。
void DialMotorConfig_Init(void)
{
    dial_motor_config.current_limit = 2000; // 拨盘电流命令限幅，原始码。
    dial_motor_config.offline_timeout_ms = 100U; // 拨盘反馈离线超时，ms。
    dial_motor_config.tx_guard_ms = 1U; // 收到回报或发出上一帧后，下次电机命令需等待的时间，ms。
    dial_motor_config.pid_control_time_s = 0.001f; // 电机 PID 每次调用使用的控制周期，s。
    DialPositionPidConfig_Init(&dial_motor_config.position);
    DialSpeedPidConfig_Init(&dial_motor_config.speed);
    DialContinuousSpeedPidConfig_Init(&dial_motor_config.continuous_speed);
}

volatile GimbalImuConfig gimbal_imu_config; // 上板 IMU 姿态与滤波参数。

// 加载云台姿态 EKF 的噪声、残差门槛和零偏收敛参数。
void GimbalAttitudeEkfConfig_Init(volatile QuaternionEkfConfig *config)
{
    if (config == NULL) { return; }
    config->quaternion_noise = 15.0f; // 四元数过程噪声，控制加速度修正权重。
    config->bias_noise = 0.001f; // 在线零偏估计的过程噪声。
    config->accel_noise = 100000.0f; // 归一化加速度的量测噪声。
    config->fading = 1.0f; // 零偏协方差遗忘系数，减小可避免过度收敛。
    config->chi_square_threshold = 1e-8f; // 收敛后的加速度残差拒绝门槛。
}

// 加载云台 IMU 更新周期、零偏采样、角速度滤波及 EKF 默认值。
void GimbalImuConfig_Init(void)
{
    gimbal_imu_config.update_period_s = 0.004f; // IMU 姿态积分使用的更新周期，s。
    gimbal_imu_config.calibration_samples = 1500U; // 静止时估计陀螺零偏的采样次数。
    gimbal_imu_config.yaw_rate_filter_alpha = 1.0f; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    gimbal_imu_config.read_timeout_ms = 12U; // 容忍单次 SPI 丢帧，连续失败再停惯性控制。
    GimbalAttitudeEkfConfig_Init(&gimbal_imu_config.attitude_ekf);
}

// 启动时统一加载上板驱动默认参数，随后由各驱动完成硬件初始化。
void UpperPeripheralConfig_InitAll(void)
{
    Motor4310Config_Init();
    CommunicationConfig_Init();
    Motor3508Config_Init();
    Motor2006Config_Init();
    DialMotorConfig_Init();
    GimbalImuConfig_Init();
}
