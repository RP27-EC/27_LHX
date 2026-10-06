#include "peripheral_config.h"
#include <stddef.h>

volatile TelecontrolConfig telecontrol_config; // 遥控接收超时参数。

// 加载 DBUS 有效遥控帧的接收超时参数。
void TelecontrolConfig_Init(void)
{
    telecontrol_config.timeout_ms = 100U; // DBUS 遥控帧离线超时，ms。
}

volatile ImuConfig imu_config; // 下板 IMU 姿态与安装方向参数。

// 加载底盘姿态 EKF 的噪声、残差门槛和零偏收敛参数。
void ChassisAttitudeEkfConfig_Init(volatile QuaternionEkfConfig *config)
{
    if (config == NULL) { return; }
    config->quaternion_noise = 15.0f; // 四元数过程噪声，控制加速度修正权重。
    config->bias_noise = 0.001f; // 在线零偏估计的过程噪声。
    config->accel_noise = 100000.0f; // 归一化加速度的量测噪声。
    config->fading = 1.0f; // 零偏协方差遗忘系数，减小可避免过度收敛。
    config->chi_square_threshold = 1e-8f; // 收敛后的加速度残差拒绝门槛。
}

// 加载底盘 IMU 周期、零偏标定、安装方向、角速度滤波及 EKF 默认值。
void ImuConfig_Init(void)
{
    imu_config.update_period_s = 0.004f; // IMU 姿态积分使用的更新周期，s。
    imu_config.gyro_calibration_samples = 500U; // 陀螺零偏采样次数。
    imu_config.yaw_rate_filter_alpha = 0.20f; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    imu_config.gyro_x_sign = (-1.0f); // 陀螺 X 轴安装方向系数。
    imu_config.gyro_y_sign = (-1.0f); // 陀螺 Y 轴安装方向系数。
    imu_config.gyro_z_sign = 1.0f; // 陀螺 Z 轴安装方向系数。
    imu_config.accel_x_sign = (-1.0f); // 加速度计 X 轴安装方向系数。
    imu_config.accel_y_sign = (-1.0f); // 加速度计 Y 轴安装方向系数。
    imu_config.accel_z_sign = 1.0f; // 加速度计 Z 轴安装方向系数。
    ChassisAttitudeEkfConfig_Init(&imu_config.attitude_ekf);
}

volatile Motor3508Config motor3508_config; // 电机速度、位置闭环参数。

// 按电机 ID 恢复本轮参数；各轮默认值在这里分别调整。
void Motor3508WheelConfig_Init(uint8_t motor_id)
{
    static const Motor3508WheelConfig defaults[MOTOR3508_WHEEL_COUNT] = {
        { // ID1，反馈帧 0x201。
            .position = {.kp = 2.5f, .ki = 2.0f, .kd = 0.0f,
                         .integral_limit = 500.0f, .output_limit = 3500.0f},
            .speed = {.kp = 12.0f, .ki = 3.0f, .kd = 0.0f,
                      .integral_limit = 2000.0f, .output_limit = 15000.0f},
            .feedforward = {.fixed_current_raw = 0.0f, .target_deadband_rpm = 1.0f},
        },
        { // ID2，反馈帧 0x202。
            .position = {.kp = 2.5f, .ki = 2.0f, .kd = 0.0f,
                         .integral_limit = 500.0f, .output_limit = 3500.0f},
            .speed = {.kp = 12.0f, .ki = 3.0f, .kd = 0.0f,
                      .integral_limit = 2000.0f, .output_limit = 15000.0f},
            .feedforward = {.fixed_current_raw = 0.0f, .target_deadband_rpm = 100.0f},
        },
        { // ID3，反馈帧 0x203。
            .position = {.kp = 2.5f, .ki = 2.0f, .kd = 0.0f,
                         .integral_limit = 500.0f, .output_limit = 3500.0f},
            .speed = {.kp = 12.0f, .ki = 3.0f, .kd = 0.0f,
                      .integral_limit = 2000.0f, .output_limit = 15000.0f},
            .feedforward = {.fixed_current_raw = 0.0f, .target_deadband_rpm = 1.0f},
        },
        { // ID4，反馈帧 0x204。
            .position = {.kp = 2.5f, .ki = 2.0f, .kd = 0.0f,
                         .integral_limit = 500.0f, .output_limit = 3500.0f},
            .speed = {.kp = 12.0f, .ki = 3.0f, .kd = 0.0f,
                      .integral_limit = 2000.0f, .output_limit = 15000.0f},
            .feedforward = {.fixed_current_raw = 0.0f, .target_deadband_rpm = 1.0f},
        },
    };
    if (motor_id < 1U || motor_id > MOTOR3508_WHEEL_COUNT) { return; }
    motor3508_config.wheel[motor_id - 1U] = defaults[motor_id - 1U];
}

// 加载共用保护参数，再分别初始化四轮 PID 和固定扭矩前馈。
void Motor3508Config_Init(void)
{
    uint8_t motor_id;
    motor3508_config.current_limit = 16384; // 底盘电机电流命令限幅，原始码。
    motor3508_config.offline_timeout_ms = 100U; // 底盘电机反馈离线超时，ms。
    motor3508_config.pid_control_time_s = 0.004f; // 电机 PID 每次调用使用的控制周期，s。
    for (motor_id = 1U; motor_id <= MOTOR3508_WHEEL_COUNT; ++motor_id)
    { Motor3508WheelConfig_Init(motor_id); }
}

volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。

// 加载板间 CAN 故障恢复间隔和云台、模式、升降反馈有效期。
void CommunicationConfig_Init(void)
{
    communication_config.retry_ms = 100U; // CAN2 Bus-Off 后的重启重试间隔，ms。
    communication_config.yaw_angle_timeout_ms = 100U; // 上板 Yaw 角度帧有效期，ms。
    communication_config.spin_state_timeout_ms = 150U; // 单次 C1 丢帧不立即中断自旋，失联仍停车。
    communication_config.lift_lock_timeout_ms = 150U; // 升降锁车请求失联后释放时间，ms。
}

volatile PowerCommunicationConfig power_communication_config; // 超电控制和状态超时参数。

// 加载超电在线超时、控制发送周期及基础开关默认值。
void PowerCommunicationConfig_Init(void)
{
    power_communication_config.offline_timeout_ms = 100U; // 两路状态帧独立判断在线。
    power_communication_config.tx_period_ms = 20U; // 超电基础控制帧发送周期，ms。
    power_communication_config.cap_enabled = true; // 默认开启超电。
    power_communication_config.turbo_enabled = false; // 超电 Turbo 模式开关。
    power_communication_config.precharge_enabled = false; // 超电预充开关，开启时清零充电功率字段。
}

// 启动时统一加载下板驱动默认参数，随后由各驱动完成硬件初始化。
void LowerPeripheralConfig_InitAll(void)
{
    TelecontrolConfig_Init();
    ImuConfig_Init();
    Motor3508Config_Init();
    CommunicationConfig_Init();
    PowerCommunicationConfig_Init();
}
