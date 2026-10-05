#ifndef GIMBAL_IMU_H
#define GIMBAL_IMU_H

#include "peripheral_config.h"

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// BMI088 经车体坐标变换后的姿态与角速度快照。
typedef struct
{
    float gyro_rad_s[3]; // 机体系三轴角速度，已去除启动零偏及 EKF 在线零偏，rad/s。
    float accel_m_s2[3]; // 机体系三轴加速度，单位 m/s²。
    float quaternion[4]; // 姿态四元数，顺序为 w、x、y、z。
    float roll_deg; // 横滚角，单位度。
    float pitch_deg; // 俯仰角，单位度。
    float yaw_deg; // 单圈航向角，范围约为 -180~180 度。
    float yaw_total_deg; // 跨越正负 180 度后的累计航向角。
    float yaw_rate_deg_s; // 转到车体坐标后的 Yaw 角速度，deg/s。
    float temperature_c; // BMI088 温度，单位摄氏度。
    float ekf_bias_rad_s[3]; // EKF 估计的剩余零偏，不含启动标定值。
    float ekf_chi_square; // 加速度残差检验值，供调试观察。
    bool ekf_accel_used; // 本次姿态更新是否接受加速度修正。
    uint32_t update_count; // 成功完成姿态更新的累计次数。
    uint8_t init_error; // 初始化阶段累计检测到的错误数。
    bool calibrated; // 陀螺仪零偏标定是否完成。
    bool online; // 最近成功读取仍在允许时限内。
} GimbalImu_Data_t;

// BMI088 使用 SPI1；SPI 与两个片选引脚须在硬件初始化中配置。
HAL_StatusTypeDef GimbalImu_Init(void);
bool GimbalImu_Update(void);
bool GimbalImu_Get(GimbalImu_Data_t *data);

extern volatile GimbalImu_Data_t gimbal_imu;


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const volatile GimbalImu_Data_t *sample; // 姿态和角速度结果；控制读取用 ops.get。
} GimbalImuModuleDataRefs;

typedef struct
{
    volatile GimbalImuConfig *config; // 当前可调驱动参数。
    GimbalImuModuleDataRefs data; // 反馈与解析数据引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get)(GimbalImu_Data_t *data); // 复制当前数据快照。

    // 状态维护。
    bool (*update)(void); // 执行一次状态更新。
} GimbalImuModule;

extern const GimbalImuModule gimbal_imu_driver; // 模块统一访问入口。

#endif // GIMBAL_IMU_H
