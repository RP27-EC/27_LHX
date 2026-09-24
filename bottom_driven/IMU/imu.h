#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* BMI088 原始数据、四元数姿态和底盘角速度的统一快照。 */
typedef struct
{
    float gyro_rad_s[3];             /* 机体系三轴角速度，单位 rad/s。 */
    float accel_m_s2[3];             /* 机体系三轴加速度，单位 m/s²。 */
    float linear_accel_world_m_s2[3];/* 去重力后的世界系三轴线加速度。 */
    float quaternion[4];             /* 姿态四元数，顺序为 w、x、y、z。 */
    float roll_deg;                  /* 横滚角，单位度。 */
    float pitch_deg;                 /* 俯仰角，单位度。 */
    float yaw_deg;                   /* 单圈航向角，约为 -180~180 度。 */
    float yaw_total_deg;             /* 跨圈累计航向角，单位度。 */
    float yaw_rate_deg_s;            /* 底盘 Yaw 轴角速度，单位度每秒。 */
    float temperature_c;             /* BMI088 温度，单位摄氏度。 */
    uint32_t update_count;           /* 成功更新姿态的累计次数。 */
    uint8_t init_error;              /* 初始化阶段累计检测到的错误数。 */
    bool calibrated;                 /* 陀螺仪零偏标定是否完成。 */
    bool online;                     /* 最近一次传感器读取是否成功。 */
} ChassisImu_Data_t;

/* BMI088 使用 SPI2；SPI 与两个片选引脚须在硬件初始化中配置。 */
HAL_StatusTypeDef ChassisImu_Init(void);

/* 固定周期调用：读取 BMI088、校准零偏并更新完整姿态。 */
bool ChassisImu_Update(void);

/* 原子复制状态；返回值表示传感器已校准且当前在线。 */
bool ChassisImu_Get(ChassisImu_Data_t *data);
bool ChassisImu_GetYawRate(float *yaw_rate_deg_s);

extern volatile ChassisImu_Data_t chassis_imu;

#endif /* CHASSIS_IMU_H */
