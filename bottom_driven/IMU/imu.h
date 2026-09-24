#ifndef GIMBAL_IMU_H
#define GIMBAL_IMU_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* BMI088 经车体坐标变换后的姿态与角速度快照。 */
typedef struct
{
    float gyro_rad_s[3];  /* 机体系三轴角速度，单位 rad/s。 */
    float accel_m_s2[3];  /* 机体系三轴加速度，单位 m/s²。 */
    float quaternion[4];  /* 姿态四元数，顺序为 w、x、y、z。 */
    float roll_deg;       /* 横滚角，单位度。 */
    float pitch_deg;      /* 俯仰角，单位度。 */
    float yaw_deg;        /* 单圈航向角，范围约为 -180~180 度。 */
    float yaw_total_deg;  /* 跨越正负 180 度后的累计航向角。 */
    float yaw_rate_deg_s; /* 转到车体坐标后的 Yaw 角速度，deg/s。 */
    float temperature_c;  /* BMI088 温度，单位摄氏度。 */
    uint32_t update_count;/* 成功完成姿态更新的累计次数。 */
    uint8_t init_error;   /* 初始化阶段累计检测到的错误数。 */
    bool calibrated;      /* 陀螺仪零偏标定是否完成。 */
    bool online;          /* 最近一次传感器读取是否成功。 */
} GimbalImu_Data_t;

/* BMI088 使用 SPI1；SPI 与两个片选引脚须在硬件初始化中配置。 */
HAL_StatusTypeDef GimbalImu_Init(void);
bool GimbalImu_Update(void);
bool GimbalImu_Get(GimbalImu_Data_t *data);

extern volatile GimbalImu_Data_t gimbal_imu;

#endif /* GIMBAL_IMU_H */
