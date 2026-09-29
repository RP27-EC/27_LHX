#ifndef DOWN_PERIPHERAL_CONFIG_H
#define DOWN_PERIPHERAL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

// 启动时加载默认值；运行时可修改配置变量。
typedef struct
{
    uint32_t timeout_ms; // 最后有效 DBUS 帧的离线超时，ms。
} TelecontrolConfig;
extern volatile TelecontrolConfig telecontrol_config;
void TelecontrolConfig_Init(void);

typedef struct
{
    float update_period_s; // 姿态积分使用的周期，1 ms。
    uint32_t gyro_calibration_samples; // 启动时的静止陀螺零偏采样次数；重调需重新初始化 IMU。
    float attitude_kp; // 加速度重力方向对姿态的比例校正增益。
    float attitude_ki; // 姿态误差积分校正增益。
    float yaw_rate_filter_alpha; // 新角速度样本权重；滤波值+=权重×(新值-旧值)。
    float gyro_x_sign; // 陀螺 X 轴方向系数。
    float gyro_y_sign; // 陀螺 Y 轴方向系数。
    float gyro_z_sign; // 陀螺 Z 轴方向系数。
    float accel_x_sign; // 加速度 X 轴方向系数。
    float accel_y_sign; // 加速度 Y 轴方向系数。
    float accel_z_sign; // 加速度 Z 轴方向系数。
} ImuConfig;
extern volatile ImuConfig imu_config;
void ImuConfig_Init(void);

typedef struct
{
    int32_t current_limit; // C620 四电机电流命令原始码的绝对值上限。
    uint32_t offline_timeout_ms; // 任一底盘电机反馈超时即停车。
    float position_kp; // 计数误差到目标 rpm 的比例增益。
    float position_ki; // 位置误差积分增益。
    float position_kd; // 位置误差微分增益；0 为关闭。
    float position_integral_limit; // 位置环积分项绝对值上限。
    float position_output_limit; // 位置环输出目标速度绝对值上限，rpm。
    float speed_kp; // rpm 误差到电流码的比例增益。
    float speed_ki; // rpm 误差积分增益。
    float speed_kd; // rpm 误差微分增益；0 为关闭。
    float speed_integral_limit; // 速度环积分项绝对值上限。
    float speed_output_limit; // PID 电流码输出上限，另受 CURRENT_LIMIT 约束。
    float pid_control_time_s; // PID 每次调用间隔，1 ms。
} Motor3508Config;
extern volatile Motor3508Config motor3508_config;
void Motor3508Config_Init(void);

typedef struct
{
    uint32_t retry_ms; // CAN2 Bus-Off 后两次 Stop/Start 尝试的最短间隔。
    uint32_t yaw_angle_timeout_ms; // C1 云台机械角度帧的有效期。
    uint32_t spin_state_timeout_ms; // C1 自旋模式位的独立有效期；明确禁止仍立即生效。
    uint32_t lift_lock_timeout_ms; // C2 升降锁车请求的有效期。
} CommunicationConfig;
extern volatile CommunicationConfig communication_config;
void CommunicationConfig_Init(void);

typedef struct
{
    uint32_t offline_timeout_ms; // 电容与无线充状态帧的离线超时，ms。
    uint32_t tx_period_ms; // 0x222 基础控制帧发送周期，ms。
    uint8_t chassis_power_buffer; // 0x222 字节 0；未接裁判系统时使用固定值。
    uint16_t chassis_power_limit; // 0x222 字节 1~2；固定协议值，不参与动态限功率。
    int16_t cap_power_out_limit; // 0x222 字节 3~4；模板使用负值。
    uint16_t cap_power_in_limit; // 0x222 字节 5~6；固定协议值。
    bool cap_enabled; // 0x222 字节 7 bit0，超电基础开关。
    bool turbo_enabled; // 0x222 字节 7 bit1，Turbo 模式。
    bool precharge_enabled; // 0x222 字节 7 bit2，预充模式。
} PowerCommunicationConfig;
extern volatile PowerCommunicationConfig power_communication_config;
void PowerCommunicationConfig_Init(void);

void LowerPeripheralConfig_InitAll(void);

#endif // DOWN_PERIPHERAL_CONFIG_H
