#ifndef UP_PERIPHERAL_CONFIG_H
#define UP_PERIPHERAL_CONFIG_H

#include <stdint.h>
#include "quaternion_ekf.h"

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    float kp; // 位置环比例增益。
    float ki; // 位置环积分增益。
    float kd; // 位置环微分增益。
    float integral_limit; // 位置环积分限幅。
    float output_limit; // 位置环输出速度限幅。
} Motor4310PositionPidConfig;

typedef struct
{
    float kp; // 速度环比例增益。
    float ki; // 速度环积分增益。
    float kd; // 速度环微分增益。
    float integral_limit; // 速度环积分限幅。
    float output_limit; // 速度环输出转矩限幅。
} Motor4310SpeedPidConfig;

typedef struct
{
    Motor4310PositionPidConfig position; // 位置环参数。
    Motor4310SpeedPidConfig speed; // 速度环参数。
} Motor4310_PidProfile_t;

typedef struct
{
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t disable_retry_ms; // 保险状态下周期重发失能命令的间隔。
    float control_period_s; // 电机 PID 每次调用使用的控制周期，s。
    Motor4310_PidProfile_t yaw_hold; // Yaw 机械保持串级 PID 参数。
    Motor4310_PidProfile_t yaw_turn; // 调头独立串级 PID；位置输出是速度原始码。
    float yaw_speed_feedforward_raw; // Yaw 速度目标非零时叠加的固定转矩码幅值。
    float yaw_speed_feedforward_deadband_raw; // 驱动速度环目标的前馈启用死区，原始码。
    Motor4310_PidProfile_t pitch; // Pitch 独立位置-速度串级 PID 参数。
} Motor4310Config;
// 反馈定义：bottom_driven/motor/motor4310/motor4310.h，Motor4310_Data_t。
// Watch：motor4310_data[MOTOR4310_PITCH] / motor4310_data[MOTOR4310_YAW]。
// 包含位置、速度、转矩原值及累计角度；控制读取用 Motor4310_GetFeedback()。
// 模块入口：motor4310；参数用 .config，状态用 .data，操作用 .ops。
extern volatile Motor4310Config motor4310_config; // 云台电机闭环与通信参数。
void Motor4310Config_Init(void);

typedef struct
{
    uint32_t timeout_ms; // 从 D1 接收起计算拼帧和遥控数据有效期，超时断控。
    uint32_t yaw_rate_timeout_ms; // D4 底盘角速度帧的有效期。
} CommunicationConfig;
// 数据定义：bottom_driven/communication/board/communication.h。
// 遥控解析：Communication_RcControl_t，Watch 为 communication_rc；用 Communication_RC_Get() 读取。
// CAN 原帧：Communication_CanRxFrame_t，用 Communication_CAN_GetLatest() 按 D1~D6 读取。
// 热量解析：Communication_HeatSnapshot_t，用 Communication_GetHeatSnapshot() 读取 D6。
// 底盘角速度：Communication_CAN_GetChassisYawRateState() 返回 deg/s 与接收时刻。
// 模块入口：board_link；参数用 .config，状态用 .data，操作用 .ops。
extern volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。
void CommunicationConfig_Init(void);

typedef struct
{
    float kp; // rpm 误差到电流码的比例增益。
    float ki; // rpm 误差积分增益。
    float kd; // rpm 误差微分增益。
    float integral_limit; // 速度环积分项绝对值上限。
    float output_limit; // PID 电流码输出上限，另受 CURRENT_LIMIT 约束。
} Motor3508SpeedPidConfig;

typedef struct
{
    int32_t current_limit; // 发给 C620 的电流码绝对值上限。
    float max_speed_rpm; // 目标速度绝对值上限，rpm。
    float left_direction; // 左轮目标速度方向系数。
    float right_direction; // 右轮反转系数，和左轮等大反向。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    Motor3508SpeedPidConfig speed; // 速度环参数。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} Motor3508Config;
// 反馈定义：bottom_driven/motor/motor3508/motor3508.h，Motor3508_Feedback_t。
// Watch：motor3508_feedback[]，数组下标按电机 ID 顺序；控制读取用 Motor3508_GetFeedback()。
// encoder 为单圈原值，speed_rpm 为转子转速，current_raw 为电调反馈电流码。
// 模块入口：motor3508；参数用 .config，状态用 .data，操作用 .ops。
extern volatile Motor3508Config motor3508_config; // 电机闭环与命令保护参数。
void Motor3508Config_Init(void);

typedef struct
{
    float kp; // 速度比例增益。
    float ki; // 速度积分增益。
    float kd; // 速度微分增益。
    float integral_limit; // 积分限幅。
    float output_limit; // 速度环输出限幅。
} Motor2006SpeedPidConfig;

typedef struct
{
    int32_t current_limit; // C610 电流原始码绝对值限幅。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t command_timeout_ms; // 非零电流命令超过此时长未更新则自动清零，ms。
    float torque_constant; // 转矩电流换算系数。
    Motor2006SpeedPidConfig speed; // 速度环参数。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} Motor2006Config;
// 反馈定义：bottom_driven/motor/motor2006/motor2006.h，Motor2006_Feedback_t。
// Watch：motor2006_feedback；控制读取用 Motor2006_GetFeedback()。
// encoder_total 为相对首帧的转子累计计数；输出轴角度用 Motor2006_GetOutputAngleDeg()。
// 模块入口：motor2006；参数用 .config，状态用 .data，操作用 .ops。
extern volatile Motor2006Config motor2006_config; // 升降电机速度环与命令保护参数。
void Motor2006Config_Init(void);

typedef struct
{
    float kp; // 位置计数误差到目标 deg/s 的比例增益。
    float ki; // 位置环积分增益；0 为关闭。
    float kd; // 位置环微分增益；0 为关闭。
    float integral_limit; // 位置环积分项限幅；0 不保留积分贡献。
    float speed_limit_dps; // 位置环输出目标速度上限，deg/s。
} DialPositionPidConfig;

typedef struct
{
    float kp; // 单发位置内环速度比例增益。
    float ki; // 单发位置内环速度积分增益。
    float kd; // 单发位置内环速度微分增益。
    float integral_limit; // 单发、连发速度环共用的积分项限幅。
    float output_limit; // 两个速度环输出电流码上限。
} DialSpeedPidConfig;

typedef struct
{
    float kp; // 连发独立速度环比例增益。
    float ki; // 连发独立速度环积分增益。
    float kd; // 连发独立速度环微分增益。
} DialContinuousSpeedPidConfig;

typedef struct
{
    int32_t current_limit; // 0xA1 电流命令的最终绝对值限幅。
    uint32_t offline_timeout_ms; // 设备反馈超过此时长未更新则判离线，ms。
    uint32_t tx_guard_ms; // 收到回报或发出上一帧后，下次电机命令需等待的时间，ms。
    DialPositionPidConfig position; // 位置环参数。
    DialSpeedPidConfig speed; // 速度环参数。
    DialContinuousSpeedPidConfig continuous_speed; // 连发速度环参数。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} DialMotorConfig;
// 反馈定义：bottom_driven/motor/dial_motor/dial_motor.h，DialMotor_Feedback_t。
// Watch：dial_motor_feedback；控制读取用 DialMotor_GetFeedback()。
// encoder 为单圈原值，encoder_total / position_deg 为相对首帧的累计位置，speed_dps 为度/s。
// 发送诊断：DialMotor_TxDiagnostics_t，Watch 为 dial_motor_tx_diagnostics。
// 模块入口：dial_motor；参数用 .config，状态用 .data，操作用 .ops。
extern volatile DialMotorConfig dial_motor_config; // 拨盘电机闭环与通信参数。
void DialMotorConfig_Init(void);

typedef struct
{
    float update_period_s; // IMU 姿态积分使用的更新周期，s。
    uint32_t calibration_samples; // 静止时估计陀螺零偏的采样次数。
    QuaternionEkfConfig attitude_ekf; // 姿态 EKF 的过程噪声、量测噪声与残差门槛。
    float yaw_rate_filter_alpha; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    uint32_t read_timeout_ms; // 连续未读到 IMU 超过此时间才判离线。
} GimbalImuConfig;
// 数据定义：bottom_driven/IMU/imu.h，GimbalImu_Data_t；Watch 为 gimbal_imu，用 GimbalImu_Get() 读取。
// gyro_rad_s / accel_m_s2 为换算后的角速度、加速度；四元数及角度为 EKF 姿态结果。
// yaw_rate_deg_s 为控制用 Yaw 角速度，ekf_bias_rad_s 为在线零偏估计。
// 模块入口：gimbal_imu_driver；参数用 .config，状态用 .data，操作用 .ops。
extern volatile GimbalImuConfig gimbal_imu_config; // 上板 IMU 姿态与滤波参数。
void GimbalImuConfig_Init(void);

// 分类默认值，可单独恢复某一组参数。
void Motor4310YawHoldConfig_Init(volatile Motor4310_PidProfile_t *config);
void Motor4310YawTurnConfig_Init(volatile Motor4310_PidProfile_t *config);
void Motor4310PitchConfig_Init(volatile Motor4310_PidProfile_t *config);
void Motor3508SpeedPidConfig_Init(volatile Motor3508SpeedPidConfig *config);
void Motor2006SpeedPidConfig_Init(volatile Motor2006SpeedPidConfig *config);
void DialPositionPidConfig_Init(volatile DialPositionPidConfig *config);
void DialSpeedPidConfig_Init(volatile DialSpeedPidConfig *config);
void DialContinuousSpeedPidConfig_Init(volatile DialContinuousSpeedPidConfig *config);
void GimbalAttitudeEkfConfig_Init(volatile QuaternionEkfConfig *config);

void UpperPeripheralConfig_InitAll(void);

#endif // UP_PERIPHERAL_CONFIG_H
