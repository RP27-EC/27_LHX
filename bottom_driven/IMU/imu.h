#ifndef GIMBAL_IMU_H
#define GIMBAL_IMU_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* BMI088 经模板坐标变换后的姿态与角速度快照。 */
typedef struct
{
    float gyro_rad_s[3];
    float accel_m_s2[3];
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
} GimbalImu_Data_t;

/* 按模板硬件自行配置 SPI1 与 BMI088 片选引脚。 */
HAL_StatusTypeDef GimbalImu_Init(void);
bool GimbalImu_Update(void);
bool GimbalImu_Get(GimbalImu_Data_t *data);

extern volatile GimbalImu_Data_t gimbal_imu;

#endif /* GIMBAL_IMU_H */
