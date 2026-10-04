#ifndef DOWN_PERIPHERAL_CONFIG_H
#define DOWN_PERIPHERAL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "quaternion_ekf.h"

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t timeout_ms; // 最后有效 DBUS 帧的离线超时，ms。
} TelecontrolConfig;
// 数据定义：telecontrol/telecontrol.h，RC_ctrl_t；Watch 为 rc_ctrl。
// rc.ch[] 为去中值后的通道，rc.s[] 为拨杆档位，mouse / key 为键鼠解析结果。
// 原始 DBUS 数据在 sbus_rx_buf[][]；任务用 RC_TakeFrame() 取帧，再用 RC_ParseFrame() 解析。
extern volatile TelecontrolConfig telecontrol_config; // 遥控接收超时参数。
void TelecontrolConfig_Init(void);

typedef struct
{
    float update_period_s; // IMU 姿态积分使用的更新周期，s。
    uint32_t gyro_calibration_samples; // 启动时的静止陀螺零偏采样次数；重调需重新初始化 IMU。
    QuaternionEkfConfig attitude_ekf; // 姿态 EKF 的过程噪声、量测噪声与残差门槛。
    float yaw_rate_filter_alpha; // Yaw 角速度低通的新样本权重，越大响应越快、滤波越弱。
    float gyro_x_sign; // 陀螺 X 轴方向系数。
    float gyro_y_sign; // 陀螺 Y 轴方向系数。
    float gyro_z_sign; // 陀螺 Z 轴方向系数。
    float accel_x_sign; // 加速度 X 轴方向系数。
    float accel_y_sign; // 加速度 Y 轴方向系数。
    float accel_z_sign; // 加速度 Z 轴方向系数。
} ImuConfig;
// 数据定义：IMU/imu.h，ChassisImu_Data_t；Watch 为 chassis_imu，用 ChassisImu_Get() 读取。
// gyro_rad_s / accel_m_s2 为换算后的角速度、加速度；四元数及角度为 EKF 姿态结果。
// 底盘 Yaw 角速度用 ChassisImu_GetYawRate() 读取，单位 deg/s。
extern volatile ImuConfig imu_config; // 下板 IMU 姿态与安装方向参数。
void ImuConfig_Init(void);

typedef struct
{
    float kp; // 计数误差到目标 rpm 的比例增益。
    float ki; // 位置误差积分增益。
    float kd; // 位置误差微分增益；0 为关闭。
    float integral_limit; // 位置环积分项绝对值上限。
    float output_limit; // 位置环输出目标速度绝对值上限，rpm。
} Motor3508PositionPidConfig;

typedef struct
{
    float kp; // rpm 误差到电流码的比例增益。
    float ki; // rpm 误差积分增益。
    float kd; // rpm 误差微分增益；0 为关闭。
    float integral_limit; // 速度环积分项绝对值上限。
    float output_limit; // PID 电流码输出上限，另受 CURRENT_LIMIT 约束。
} Motor3508SpeedPidConfig;

typedef struct
{
    int32_t current_limit; // C620 四电机电流命令原始码的绝对值上限。
    uint32_t offline_timeout_ms; // 任一底盘电机反馈超时即停车。
    Motor3508PositionPidConfig position; // 位置环参数。
    Motor3508SpeedPidConfig speed; // 速度环参数。
    float pid_control_time_s; // 电机 PID 每次调用使用的控制周期，s。
} Motor3508Config;
// 反馈定义：motor/motor3508.h，Motor3508_Feedback。
// 数据保存在 motor3508.c 内的 motor_feedback[]；用 Motor3508_GetFeedback() 按电机 ID 复制。
// encoder 为单圈原值，encoder_total / position_deg 为相对首帧的累计转子位置。
// speed_rpm 为转子转速，current_raw 为电调反馈电流码。
// CAN 总线诊断：communication/chassis_can.h，ChassisCanState；Watch 为 chassis_can_state。
extern volatile Motor3508Config motor3508_config; // 电机速度、位置闭环参数。
void Motor3508Config_Init(void);

typedef struct
{
    uint32_t retry_ms; // CAN2 Bus-Off 后两次 Stop/Start 尝试的最短间隔。
    uint32_t yaw_angle_timeout_ms; // C1 云台机械角度帧的有效期。
    uint32_t spin_state_timeout_ms; // C1 自旋模式位的独立有效期；明确禁止仍立即生效。
    uint32_t lift_lock_timeout_ms; // C2 升降锁车请求的有效期。
} CommunicationConfig;
// 原帧定义：communication/communication.h，Communication_RxFrame。
// C1/C2 缓存在 communication.c 内的 communication_rx_c1 / communication_rx_c2。
// 原帧用 Communication_GetRxFrame() 复制；Yaw、调头许可用 Communication_GetYawState() 解码。
// 小陀螺许可用 Communication_GetSpinState()，升降锁车用 Communication_GetLiftLock() 读取。
extern volatile CommunicationConfig communication_config; // 板间通信重试和超时参数。
void CommunicationConfig_Init(void);

typedef struct
{
    uint32_t offline_timeout_ms; // 电容与无线充状态帧的离线超时，ms。
    uint32_t tx_period_ms; // 0x222 基础控制帧发送周期，ms。
    bool cap_enabled; // 0x222 字节 7 bit0，超电基础开关。
    bool turbo_enabled; // 超电 Turbo 模式开关。
    bool precharge_enabled; // 超电预充开关，开启时清零充电功率字段。
} PowerCommunicationConfig;
// 数据定义：communication/power_communication.h，PowerCommunicationState。
// Watch：power_communication_state；控制读取用 PowerCommunication_GetSnapshot()。
// capacitor 类型为 CapacitorStatus，保留原帧与原值，voltage_v / current_a 为换算后的 V / A。
// capacitor.chassis_power_raw 为超电回传的底盘实测功率，当前协议按 W 使用。
// wireless 类型为 WirelessChargeStatus，charging_power_w 为换算后的充电功率，W。
// control 保存最近成功发送的超电控制帧及发送统计。
extern volatile PowerCommunicationConfig power_communication_config; // 超电控制和状态超时参数。
void PowerCommunicationConfig_Init(void);

// 裁判数据引用：referee/referee.h，RefereeState_t；Watch 为 referee_state。
// info 类型为 RefereeInfo_t，汇总各命令解析结果；message 为 RefereeMessageStatus_t 数组，记录有效性和新鲜度。
// info.robot_status 为 RefereeRobotStatus_t，包含底盘功率上限和电源许可。
// info.power_heat_data 为 RefereeWire_power_heat_data_t，包含裁判功率、缓冲能量与枪口热量字段。
// 热量快照为 RefereeHeatSnapshot_t，用 Referee_GetHeatSnapshot() 同时读取热量、上限、冷却与许可。
// 完整数据用 Referee_GetState() 复制；diagnostics 类型为 RefereeDiagnostics_t，记录 CRC 与解析统计。
// 串口诊断：referee/referee_uart.h，RefereeUartDiagnostics_t；Watch 为 referee_uart_diagnostics。
// 原始串口字节在 referee_uart_dma_buffer[]，由 RefereeUart_Process() 交给裁判解析器。

// 分类默认值，可单独恢复某一组参数。
void ChassisAttitudeEkfConfig_Init(volatile QuaternionEkfConfig *config);
void Motor3508PositionPidConfig_Init(volatile Motor3508PositionPidConfig *config);
void Motor3508SpeedPidConfig_Init(volatile Motor3508SpeedPidConfig *config);

void LowerPeripheralConfig_InitAll(void);

#endif // DOWN_PERIPHERAL_CONFIG_H
