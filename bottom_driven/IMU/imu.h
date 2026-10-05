#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include "peripheral_config.h"

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// BMI088 原始数据、四元数姿态和底盘角速度的统一快照。
typedef struct
{
    float gyro_rad_s[3]; // 机体系三轴角速度，已去除启动零偏及 EKF 在线零偏，rad/s。
    float accel_m_s2[3]; // 机体系三轴加速度，单位 m/s²。
    float linear_accel_world_m_s2[3]; // 去重力后的世界系三轴线加速度。
    float quaternion[4]; // 姿态四元数，顺序为 w、x、y、z。
    float roll_deg; // 横滚角，单位度。
    float pitch_deg; // 俯仰角，单位度。
    float yaw_deg; // 单圈航向角，约为 -180~180 度。
    float yaw_total_deg; // 跨圈累计航向角，单位度。
    float yaw_rate_deg_s; // 底盘 Yaw 轴角速度，单位度每秒。
    float temperature_c; // BMI088 温度，单位摄氏度。
    float ekf_bias_rad_s[3]; // EKF 估计的剩余零偏，不含启动标定值。
    float ekf_chi_square; // 加速度残差检验值，供调试观察。
    bool ekf_accel_used; // 本次姿态更新是否接受加速度修正。
    uint32_t update_count; // 成功更新姿态的累计次数。
    uint8_t init_error; // 初始化阶段累计检测到的错误数。
    bool calibrated; // 陀螺仪零偏标定是否完成。
    bool online; // 最近一次传感器读取是否成功。
} ChassisImu_Data_t;

// BMI088 使用 SPI2；SPI 与两个片选引脚须在硬件初始化中配置。
HAL_StatusTypeDef ChassisImu_Init(void);

// 固定周期调用：读取 BMI088、校准零偏并更新完整姿态。
bool ChassisImu_Update(void);

// 原子复制状态；返回值表示传感器已校准且当前在线。
bool ChassisImu_Get(ChassisImu_Data_t *data);
bool ChassisImu_GetYawRate(float *yaw_rate_deg_s);

extern volatile ChassisImu_Data_t chassis_imu;


// 模块入口引用当前驱动数据；控制读取使用模块的快照接口。
typedef struct
{
    const volatile ChassisImu_Data_t *sample; // 姿态和角速度结果；控制读取用 ops.get。
} ChassisImuModuleDataRefs;

typedef struct
{
    volatile ImuConfig *config; // 当前可调驱动参数。
    ChassisImuModuleDataRefs data; // 反馈与解析数据引用。

    // 初始化。
    HAL_StatusTypeDef (*init)(void); // 初始化模块。

    // 数据读取与在线检查。
    bool (*get)(ChassisImu_Data_t *data); // 复制当前数据快照。
    bool (*get_yaw_rate)(float *yaw_rate_deg_s); // 读取底盘 Yaw 角速度。

    // 状态维护。
    bool (*update)(void); // 执行一次状态更新。
} ChassisImuModule;

extern const ChassisImuModule chassis_imu_driver; // 模块统一访问入口。

#endif // CHASSIS_IMU_H
