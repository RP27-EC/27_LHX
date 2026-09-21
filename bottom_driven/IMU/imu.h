#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* BMI088 原始数据、四元数姿态和底盘角速度的统一快照。 */
typedef struct
{
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float linear_accel_world_m_s2[3];
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float yaw_total_deg;
    float yaw_rate_deg_s;
    float temperature_c;
    uint32_t update_count;
    uint8_t init_error;
    bool calibrated;
    bool online;
} ChassisImu_Data_t;

/* 自行配置模板硬件使用的 SPI2 和 BMI088 片选引脚。 */
HAL_StatusTypeDef ChassisImu_Init(void);

/* 固定周期调用：读取 BMI088、校准零偏并更新完整姿态。 */
bool ChassisImu_Update(void);

/* 原子复制状态；返回值表示传感器已校准且当前在线。 */
bool ChassisImu_Get(ChassisImu_Data_t *data);
bool ChassisImu_GetYawRate(float *yaw_rate_deg_s);

extern volatile ChassisImu_Data_t chassis_imu;

#endif /* CHASSIS_IMU_H */
