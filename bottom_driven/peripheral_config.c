#include "peripheral_config.h"

volatile Motor4310Config motor4310_config; // 云台电机闭环与通信参数。

void Motor4310Config_Init(void)
{
    // 公共：在线、失能与控制周期
    motor4310_config.offline_timeout_ms = 100U; // 电机反馈离线超时，ms。
    motor4310_config.disable_retry_ms = 50U; // 失能命令重发间隔，ms。
    motor4310_config.control_period_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。

    // Yaw：机械保持位置速度环与驱动固定前馈
    motor4310_config.speed_kp = 1.7f; // Yaw 机械保持速度环比例增益。
    motor4310_config.speed_ki = 0.2f; // 4310 速度环积分增益。
    motor4310_config.speed_kd = 0.001f; // 4310 速度环微分增益。
    motor4310_config.speed_integral_limit = 350.0f; // 速度环积分项限幅。
    motor4310_config.speed_output_limit = 2047.0f; // 速度环转矩码输出限幅。
    motor4310_config.position_kp = 0.5f; // Yaw 机械保持位置环比例增益。
    motor4310_config.position_ki = 0.1f; // 4310 位置环积分增益。
    motor4310_config.position_kd = 0.002f; // 4310 位置环微分增益。
    motor4310_config.position_integral_limit = 150.0f; // 位置环积分项限幅。
    motor4310_config.position_output_limit = 300.0f; // 位置环目标速度限幅。
    motor4310_config.yaw_speed_feedforward_raw = 0.0f; // Yaw 转动时按目标方向叠加的固定转矩码。
    motor4310_config.yaw_speed_feedforward_deadband_raw = 0.0f; // 4310 原始目标速度超过此值才加前馈。

    // Yaw：独立调头，轨迹跟踪不叠加固定方向转矩。
    motor4310_config.yaw_turn_pid.position_kp = 0.8f; // 调头独立位置环比例增益。
    motor4310_config.yaw_turn_pid.position_ki = 0.0f; // 调头独立位置环积分增益。
    motor4310_config.yaw_turn_pid.position_kd = 0.0f; // 调头独立位置环微分增益。
    motor4310_config.yaw_turn_pid.position_integral_limit = 0.0f; // 调头独立位置环积分项限幅。
    motor4310_config.yaw_turn_pid.position_output_limit = 1500.0f; // 调头独立位置环目标速度限幅，原始码。
    motor4310_config.yaw_turn_pid.speed_kp = 1.7f; // 调头独立速度环比例增益。
    motor4310_config.yaw_turn_pid.speed_ki = 0.1f; // 调头独立速度环积分增益。
    motor4310_config.yaw_turn_pid.speed_kd = 0.0f; // 调头独立速度环微分增益。
    motor4310_config.yaw_turn_pid.speed_integral_limit = 0.0f; // 调头独立速度环积分项限幅。
    motor4310_config.yaw_turn_pid.speed_output_limit = 2047.0f; // 调头独立速度环转矩输出限幅，原始码。

    // Pitch：独立位置与速度环
    motor4310_config.pitch_pid.position_kp = 0.3f; // Pitch 位置环比例增益。
    motor4310_config.pitch_pid.position_ki = 0.0f; // Pitch 位置环积分增益。
    motor4310_config.pitch_pid.position_kd = 0.004f; // Pitch 位置环微分增益。
    motor4310_config.pitch_pid.position_integral_limit = 100.0f; // Pitch 位置环积分限幅。
    motor4310_config.pitch_pid.position_output_limit = 600.0f; // Pitch 目标速度限幅。
    motor4310_config.pitch_pid.speed_kp = 1.2f; // Pitch 速度环比例增益。
    motor4310_config.pitch_pid.speed_ki = 0.0f; // Pitch 速度环积分增益。
    motor4310_config.pitch_pid.speed_kd = 0.003f; // Pitch 速度环微分增益。
    motor4310_config.pitch_pid.speed_integral_limit = 200.0f; // Pitch 速度环积分限幅。
    motor4310_config.pitch_pid.speed_output_limit = 2047.0f; // Pitch 转矩码限幅。

}

volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。

void CommunicationConfig_Init(void)
{
    communication_config.timeout_ms = 100U; // 下板遥控 CAN 帧接收超时，ms。
    communication_config.yaw_rate_timeout_ms = 100U; // 下板角速度帧有效期，ms。
}

volatile Motor3508Config motor3508_config; // 电机闭环与命令保护参数。

void Motor3508Config_Init(void)
{
    motor3508_config.current_limit = 16384; // 摩擦轮电流命令限幅，原始码。
    motor3508_config.max_speed_rpm = 2000.0f; // 摩擦轮目标转速上限，rpm。
    motor3508_config.left_direction = 1.0f; // 左摩擦轮转向系数。
    motor3508_config.right_direction = (-1.0f); // 右摩擦轮转向系数。
    motor3508_config.offline_timeout_ms = 100U; // 摩擦轮反馈离线超时，ms。
    motor3508_config.speed_kp = 2.0f; // 摩擦轮速度环比例增益。
    motor3508_config.speed_ki = 1.0f; // 摩擦轮速度环积分增益。
    motor3508_config.speed_kd = 0.0f; // 摩擦轮速度环微分增益。
    motor3508_config.speed_integral_limit = 500.0f; // 速度环积分项限幅。
    motor3508_config.speed_output_limit = 5000.0f; // 速度环电流码输出限幅。
    motor3508_config.pid_control_time_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
}

volatile Motor2006Config motor2006_config; // 升降电机速度环与命令保护参数。

void Motor2006Config_Init(void)
{
    motor2006_config.current_limit = 10000; // 升降电机电流命令限幅，原始码。
    motor2006_config.offline_timeout_ms = 100U; // 升降电机反馈离线超时，ms。
    motor2006_config.command_timeout_ms = 100U; // 控制命令超时清零间隔，ms。
    motor2006_config.torque_constant = 0.18f; // 速度环转矩到电流的换算系数。
    motor2006_config.speed_kp = 12.0f; // 升降电机速度环比例增益。
    motor2006_config.speed_ki =  5.0f; // 升降电机速度环积分增益。
    motor2006_config.speed_kd = 0.0f; // 升降电机速度环微分增益。
    motor2006_config.speed_integral_limit = 0.0f; // 速度环积分项限幅。
    motor2006_config.speed_torque_output_limit = 800.0f; // 速度环转矩输出限幅。
    motor2006_config.pid_control_time_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
}

volatile DialMotorConfig dial_motor_config; // 拨盘电机闭环与通信参数。

void DialMotorConfig_Init(void)
{
    dial_motor_config.current_limit = 2000; // 拨盘电流命令限幅，原始码。
    dial_motor_config.offline_timeout_ms = 100U; // 拨盘反馈离线超时，ms。
    dial_motor_config.tx_guard_ms = 1U; // 收到回报或发出上一帧后，下次电机命令需等待的时间，ms。
    dial_motor_config.position_kp = 0.1f; // 拨盘位置环比例增益。
    dial_motor_config.position_ki = 0.0f; // 拨盘位置环积分增益。
    dial_motor_config.position_kd = 0.0f; // 拨盘位置环微分增益。
    dial_motor_config.position_integral_limit = 0.0f; // 位置环积分项限幅。
    dial_motor_config.position_speed_limit_dps = 7000.0f; // 位置环目标速度限幅，度/s。
    dial_motor_config.speed_kp = 0.06f; // 单发速度环比例增益。
    dial_motor_config.speed_ki = 0.0f; // 单发速度环积分增益。
    dial_motor_config.speed_kd = 0.0f; // 单发速度环微分增益。
    dial_motor_config.speed_integral_limit = 500.0f; // 速度环积分项限幅。
    dial_motor_config.speed_output_limit = 1500.0f; // 速度环电流码输出限幅。
    dial_motor_config.continuous_speed_kp = 0.08f; // 连发速度环比例增益。
    dial_motor_config.continuous_speed_ki = 0.0f; // 连发速度环积分增益。
    dial_motor_config.continuous_speed_kd = 0.0f; // 连发速度环微分增益。
    dial_motor_config.pid_control_time_s = 0.001f; // 电机 PID 每次调用使用的控制周期，s。
}

volatile GimbalImuConfig gimbal_imu_config; // 上板 IMU 姿态与滤波参数。

void GimbalImuConfig_Init(void)
{
    gimbal_imu_config.update_period_s = 0.004f; // IMU 姿态积分使用的更新周期，s。
    gimbal_imu_config.calibration_samples = 1000U; // 静止时估计陀螺零偏的采样次数。
    gimbal_imu_config.attitude_kp = 2.0f; // 加速度修正姿态的比例增益。
    gimbal_imu_config.attitude_ki = 0.02f; // 姿态误差积分修正增益。
    gimbal_imu_config.yaw_rate_filter_alpha = 1.0f; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    gimbal_imu_config.read_timeout_ms = 12U; // 容忍单次 SPI 丢帧，连续失败再停惯性控制。
}

void UpperPeripheralConfig_InitAll(void)
{
    Motor4310Config_Init();
    CommunicationConfig_Init();
    Motor3508Config_Init();
    Motor2006Config_Init();
    DialMotorConfig_Init();
    GimbalImuConfig_Init();
}
