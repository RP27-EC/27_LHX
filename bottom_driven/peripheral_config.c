#include "peripheral_config.h"

volatile TelecontrolConfig telecontrol_config;

void TelecontrolConfig_Init(void)
{
    telecontrol_config.timeout_ms = 100U; // DBUS 遥控帧离线超时，ms。
}

volatile ImuConfig imu_config;

void ImuConfig_Init(void)
{
    imu_config.update_period_s = 0.004f; // 下板 IMU 姿态更新周期，s。
    imu_config.gyro_calibration_samples = 500U; // 陀螺零偏采样次数。
    imu_config.attitude_kp = 2.0f; // 加速度修正姿态的比例增益。
    imu_config.attitude_ki = 0.02f; // 姿态误差积分修正增益。
    imu_config.yaw_rate_filter_alpha = 0.20f; // Yaw 角速度滤波新样本权重。
    imu_config.gyro_x_sign = (-1.0f); // 陀螺 X 轴安装方向系数。
    imu_config.gyro_y_sign = (-1.0f); // 陀螺 Y 轴安装方向系数。
    imu_config.gyro_z_sign = 1.0f; // 陀螺 Z 轴安装方向系数。
    imu_config.accel_x_sign = (-1.0f); // 加速度计 X 轴安装方向系数。
    imu_config.accel_y_sign = (-1.0f); // 加速度计 Y 轴安装方向系数。
    imu_config.accel_z_sign = 1.0f; // 加速度计 Z 轴安装方向系数。
}

volatile Motor3508Config motor3508_config;

void Motor3508Config_Init(void)
{
    motor3508_config.current_limit = 16384; // 底盘电机电流命令限幅，原始码。
    motor3508_config.offline_timeout_ms = 100U; // 底盘电机反馈离线超时，ms。
    motor3508_config.position_kp = 2.5f; // 底盘电机位置环比例增益。
    motor3508_config.position_ki = 2.0f; // 底盘电机位置环积分增益。
    motor3508_config.position_kd = 0.0f; // 底盘电机位置环微分增益。
    motor3508_config.position_integral_limit = 500.0f; // 位置环积分项限幅。
    motor3508_config.position_output_limit = 3500.0f; // 位置环目标轮速限幅，rpm。
    motor3508_config.speed_kp = 8.0f; // 底盘电机速度环比例增益。
    motor3508_config.speed_ki = 2.0f; // 底盘电机速度环积分增益。
    motor3508_config.speed_kd = 0.0f; // 底盘电机速度环微分增益。
    motor3508_config.speed_integral_limit = 1000.0f; // 速度环积分项限幅。
    motor3508_config.speed_output_limit = 10000.0f; // 速度环电流码输出限幅。
    motor3508_config.pid_control_time_s = 0.004f; // PID 控制周期，s。
}

volatile CommunicationConfig communication_config;

void CommunicationConfig_Init(void)
{
    communication_config.retry_ms = 100U; // CAN2 Bus-Off 后的重启重试间隔，ms。
    communication_config.yaw_angle_timeout_ms = 100U; // 上板 Yaw 角度帧有效期，ms。
    communication_config.spin_state_timeout_ms = 150U; // 单次 C1 丢帧不立即中断自旋，失联仍停车。
    communication_config.lift_lock_timeout_ms = 150U; // 升降锁车请求失联后释放时间，ms。
}

volatile PowerCommunicationConfig power_communication_config;

void PowerCommunicationConfig_Init(void)
{
    power_communication_config.offline_timeout_ms = 100U; // 两路状态帧独立判断在线。
    power_communication_config.tx_period_ms = 20U; // 基础控制帧每 20 ms 发送一次。
    power_communication_config.chassis_power_buffer = 0U; // 暂不接裁判系统缓冲量。
    power_communication_config.chassis_power_limit = 300U; // 固定协议值
    power_communication_config.cap_power_out_limit = -300; // 模板默认放电字段。
    power_communication_config.cap_power_in_limit = 300U; // 模板默认充电字段。
    power_communication_config.cap_enabled = true; // 默认开启超电。
    power_communication_config.turbo_enabled = false;
    power_communication_config.precharge_enabled = false;
}

void LowerPeripheralConfig_InitAll(void)
{
    TelecontrolConfig_Init();
    ImuConfig_Init();
    Motor3508Config_Init();
    CommunicationConfig_Init();
    PowerCommunicationConfig_Init();
}
