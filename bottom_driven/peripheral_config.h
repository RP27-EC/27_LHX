#ifndef UP_PERIPHERAL_CONFIG_H
#define UP_PERIPHERAL_CONFIG_H

#include <stdint.h>

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    float position_kp; // 位置环比例增益。
    float position_ki; // 位置环积分增益。
    float position_kd; // 位置环微分增益。
    float position_integral_limit; // 位置环积分限幅。
    float position_output_limit; // 位置环输出速度限幅。
    float speed_kp; // 速度环比例增益。
    float speed_ki; // 速度环积分增益。
    float speed_kd; // 速度环微分增益。
    float speed_integral_limit; // 速度环积分限幅。
    float speed_output_limit; // 速度环输出转矩限幅。
} Motor4310_PidProfile_t;

typedef struct
{
    // 公共：在线、失能与控制周期
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t disable_retry_ms; // 保险状态下周期重发失能命令的间隔。
    float control_period_s; // 电机 PID 每次调用使用的控制周期，s。

    // Yaw：机械保持位置速度环与驱动固定前馈
    float speed_kp; // 速度误差到转矩码的比例增益。
    float speed_ki; // 速度环积分增益，积分按控制周期累积。
    float speed_kd; // 速度环微分增益；0 为关闭。
    float speed_integral_limit; // 速度环积分项绝对值上限。
    float speed_output_limit; // 速度环输出转矩原始码上限。
    float position_kp; // 位置计数误差到速度目标的比例增益。
    float position_ki; // 位置环积分增益。
    float position_kd; // 位置环微分增益；0 为关闭。
    float position_integral_limit; // 位置环积分项绝对值上限。
    float position_output_limit; // 位置环速度原始码目标上限。
    Motor4310_PidProfile_t yaw_turn_pid; // 调头独立串级 PID；位置输出是速度原始码。
    float yaw_speed_feedforward_raw; // Yaw 速度目标非零时叠加的固定转矩码幅值。
    float yaw_speed_feedforward_deadband_raw; // 驱动速度环目标的前馈启用死区，原始码。

    // Pitch：独立位置与速度环
    Motor4310_PidProfile_t pitch_pid; // Pitch 独立位置-速度串级 PID 参数。

} Motor4310Config;
extern volatile Motor4310Config motor4310_config; // 云台电机闭环与通信参数。
void Motor4310Config_Init(void);

typedef struct
{
    uint32_t timeout_ms; // D1~D3 有效遥控帧超过此时长未更新，即判断控。
    uint32_t yaw_rate_timeout_ms; // D4 底盘角速度帧的有效期。
} CommunicationConfig;
extern volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。
void CommunicationConfig_Init(void);

typedef struct
{
    int32_t current_limit; // 发给 C620 的电流码绝对值上限。
    float max_speed_rpm; // 目标速度绝对值上限，rpm。
    float left_direction; // 左轮目标速度方向系数。
    float right_direction; // 右轮反转系数，和左轮等大反向。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    float speed_kp; // rpm 误差到电流码的比例增益。
    float speed_ki; // rpm 误差积分增益。
    float speed_kd; // rpm 误差微分增益。
    float speed_integral_limit; // 速度环积分项绝对值上限。
    float speed_output_limit; // PID 电流码输出上限，另受 CURRENT_LIMIT 约束。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} Motor3508Config;
extern volatile Motor3508Config motor3508_config; // 电机闭环与命令保护参数。
void Motor3508Config_Init(void);

typedef struct
{
    int32_t current_limit; // C610 电流原始码绝对值限幅。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t command_timeout_ms; // 非零电流命令超过此时长未更新则自动清零，ms。
    float torque_constant; // 转矩电流换算系数。
    float speed_kp; // 速度比例增益。
    float speed_ki; // 速度积分增益。
    float speed_kd; // 速度微分增益。
    float speed_integral_limit; // 积分限幅。
    float speed_torque_output_limit; // 速度环输出限幅。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} Motor2006Config;
extern volatile Motor2006Config motor2006_config; // 升降电机速度环与命令保护参数。
void Motor2006Config_Init(void);

typedef struct
{
    int32_t current_limit; // 0xA1 电流命令的最终绝对值限幅。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t tx_guard_ms; // 收到回报或发出上一帧后，下次电机命令需等待的时间，ms。
    float position_kp; // 位置计数误差到目标 deg/s 的比例增益。
    float position_ki; // 位置环积分增益；0 为关闭。
    float position_kd; // 位置环微分增益；0 为关闭。
    float position_integral_limit; // 位置环积分项限幅；0 不保留积分贡献。
    float position_speed_limit_dps; // 位置环输出目标速度上限，deg/s。
    float speed_kp; // 单发位置内环速度比例增益。
    float speed_ki; // 单发位置内环速度积分增益。
    float speed_kd; // 单发位置内环速度微分增益。
    float speed_integral_limit; // 单发、连发速度环共用的积分项限幅。
    float speed_output_limit; // 两个速度环输出电流码上限。
    float continuous_speed_kp; // 连发独立速度环比例增益。
    float continuous_speed_ki; // 连发独立速度环积分增益。
    float continuous_speed_kd; // 连发独立速度环微分增益。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} DialMotorConfig;
extern volatile DialMotorConfig dial_motor_config; // 拨盘电机闭环与通信参数。
void DialMotorConfig_Init(void);

typedef struct
{
    float update_period_s; // IMU 姿态积分使用的更新周期，s。
    uint32_t calibration_samples; // 静止时估计陀螺零偏的采样次数。
    float attitude_kp; // 加速度重力方向修正姿态的比例增益。
    float attitude_ki; // 姿态误差积分修正增益。
    float yaw_rate_filter_alpha; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    uint32_t read_timeout_ms; // 连续未读到 IMU 超过此时间才判离线。
} GimbalImuConfig;
extern volatile GimbalImuConfig gimbal_imu_config; // 上板 IMU 姿态与滤波参数。
void GimbalImuConfig_Init(void);

void UpperPeripheralConfig_InitAll(void);

#endif // UP_PERIPHERAL_CONFIG_H
