#include "imu.h"
#include "parameter.h"
#include "spi.h"
#include <math.h>
#include <string.h>

/* 上板 BMI088：SPI1，PA4 加速度计 CS，PB0 陀螺仪 CS。 */
#define IMU_ACCEL_CS_PORT              GPIOA
#define IMU_ACCEL_CS_PIN               GPIO_PIN_4
#define IMU_GYRO_CS_PORT               GPIOB
#define IMU_GYRO_CS_PIN                GPIO_PIN_0

#define BMI088_ACC_CHIP_ID             0x00U
#define BMI088_ACC_CHIP_ID_VALUE       0x1EU
#define BMI088_ACC_X_L                 0x12U
#define BMI088_ACC_TEMP_M              0x22U
#define BMI088_ACC_CONF                0x40U
#define BMI088_ACC_RANGE               0x41U
#define BMI088_ACC_PWR_CONF            0x7CU
#define BMI088_ACC_PWR_CTRL            0x7DU
#define BMI088_ACC_SOFTRESET           0x7EU
#define BMI088_GYRO_CHIP_ID            0x00U
#define BMI088_GYRO_CHIP_ID_VALUE      0x0FU
#define BMI088_GYRO_X_L                0x02U
#define BMI088_GYRO_RANGE              0x0FU
#define BMI088_GYRO_BANDWIDTH          0x10U
#define BMI088_GYRO_LPM1               0x11U
#define BMI088_GYRO_SOFTRESET          0x14U

#define BMI088_ACC_SENSITIVITY         0.0008974358974f
#define BMI088_GYRO_SENSITIVITY        0.0010652644360f
#define IMU_RAD_TO_DEG                 57.2957795131f

static float gyro_bias[3];        /* 标定得到的三轴陀螺仪零偏。 */
static float integral_feedback[3];/* 姿态融合中用于消除漂移的积分反馈。 */
static uint32_t last_update_ms;   /* 上一次姿态更新的毫秒时间戳。 */
static float yaw_last_deg;        /* 上一周期单圈 Yaw 角，用于跨圈判断。 */
static int32_t yaw_rounds;        /* Yaw 跨越正负 180 度的累计圈数。 */

volatile GimbalImu_Data_t gimbal_imu; /* 供控制任务和调试器读取的 IMU 快照。 */

static void imu_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    start = DWT->CYCCNT;
    ticks = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

static uint8_t imu_spi_byte(uint8_t value)
{
    uint8_t result = 0U;
    (void)HAL_SPI_TransmitReceive(&hspi1, &value, &result, 1U, 10U);
    return result;
}

static void imu_select(GPIO_TypeDef *port, uint16_t pin, bool selected)
{
    HAL_GPIO_WritePin(port, pin, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void imu_write(GPIO_TypeDef *port, uint16_t pin,
                      uint8_t reg, uint8_t value)
{
    imu_select(port, pin, true);
    (void)imu_spi_byte(reg);
    (void)imu_spi_byte(value);
    imu_select(port, pin, false);
    imu_delay_us(150U);
}

static uint8_t imu_read_reg(GPIO_TypeDef *port, uint16_t pin,
                            uint8_t reg, bool accel)
{
    uint8_t result;

    imu_select(port, pin, true);
    (void)imu_spi_byte(reg | 0x80U);
    if (accel) { (void)imu_spi_byte(0x55U); }
    result = imu_spi_byte(0x55U);
    imu_select(port, pin, false);
    return result;
}

static void imu_read_burst(GPIO_TypeDef *port, uint16_t pin, uint8_t reg,
                           bool accel, uint8_t *data, uint32_t length)
{
    uint32_t index;

    imu_select(port, pin, true);
    (void)imu_spi_byte(reg | 0x80U);
    if (accel) { (void)imu_spi_byte(0x55U); }
    for (index = 0U; index < length; index++)
    {
        data[index] = imu_spi_byte(0x55U);
    }
    imu_select(port, pin, false);
}

static bool imu_verify(GPIO_TypeDef *port, uint16_t pin,
                       uint8_t reg, uint8_t value, bool accel)
{
    imu_write(port, pin, reg, value);
    return imu_read_reg(port, pin, reg, accel) == value;
}

static uint8_t bmi088_init(void)
{
    imu_select(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN, false);
    imu_select(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, false);
    HAL_Delay(10U);

    /* BMI088 加速度计进入 SPI 模式时需要连续读取两次。 */
    (void)imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                       BMI088_ACC_CHIP_ID, true);
    if (imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                     BMI088_ACC_CHIP_ID, true) != BMI088_ACC_CHIP_ID_VALUE)
    {
        return 0x80U;
    }
    imu_write(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
              BMI088_ACC_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    (void)imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                       BMI088_ACC_CHIP_ID, true);
    if (imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                     BMI088_ACC_CHIP_ID, true) != BMI088_ACC_CHIP_ID_VALUE)
    {
        return 0x81U;
    }
    if (!imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_PWR_CTRL, 0x04U, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_PWR_CONF, 0x00U, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_CONF, 0xABU, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_RANGE, 0x00U, true))
    {
        return 0x82U;
    }

    if (imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                     BMI088_GYRO_CHIP_ID, false) != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return 0x40U;
    }
    imu_write(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
              BMI088_GYRO_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    if (imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                     BMI088_GYRO_CHIP_ID, false) != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return 0x41U;
    }
    if (!imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_RANGE, 0x00U, false) ||
        !imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_BANDWIDTH, 0x82U, false) ||
        !imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_LPM1, 0x00U, false))
    {
        return 0x42U;
    }
    return 0U;
}

static bool imu_read_sensor(float gyro[3], float accel[3], float *temperature)
{
    uint8_t data[6];
    int16_t raw;

    imu_read_burst(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                   BMI088_ACC_X_L, true, data, 6U);
    raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    accel[0] = (float)raw * BMI088_ACC_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    accel[1] = (float)raw * BMI088_ACC_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    accel[2] = (float)raw * BMI088_ACC_SENSITIVITY;

    if (imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                     BMI088_GYRO_CHIP_ID, false) != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return false;
    }
    imu_read_burst(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                   BMI088_GYRO_X_L, false, data, 6U);
    raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    gyro[0] = (float)raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    gyro[1] = (float)raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    gyro[2] = (float)raw * BMI088_GYRO_SENSITIVITY;

    imu_read_burst(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                   BMI088_ACC_TEMP_M, true, data, 2U);
    raw = (int16_t)(((uint16_t)data[0] << 3) | (data[1] >> 5));
    if (raw > 1023) { raw -= 2048; }
    *temperature = (float)raw * 0.125f + 23.0f;
    return true;
}

static void imu_update_attitude(float gx, float gy, float gz,
                                float ax, float ay, float az, float dt)
{
    float q0 = gimbal_imu.quaternion[0];
    float q1 = gimbal_imu.quaternion[1];
    float q2 = gimbal_imu.quaternion[2];
    float q3 = gimbal_imu.quaternion[3];
    float norm;
    float vx, vy, vz, ex, ey, ez;
    float nq0, nq1, nq2, nq3;
    float yaw_delta;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.1f)
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;
        integral_feedback[0] += GIMBAL_IMU_ATTITUDE_KI * ex * dt;
        integral_feedback[1] += GIMBAL_IMU_ATTITUDE_KI * ey * dt;
        integral_feedback[2] += GIMBAL_IMU_ATTITUDE_KI * ez * dt;
        gx += GIMBAL_IMU_ATTITUDE_KP * ex + integral_feedback[0];
        gy += GIMBAL_IMU_ATTITUDE_KP * ey + integral_feedback[1];
        gz += GIMBAL_IMU_ATTITUDE_KP * ez + integral_feedback[2];
    }

    nq0 = q0 + 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    nq1 = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    nq2 = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    nq3 = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;
    norm = sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
    if (norm <= 0.0f) { return; }
    q0 = nq0 / norm;
    q1 = nq1 / norm;
    q2 = nq2 / norm;
    q3 = nq3 / norm;
    gimbal_imu.quaternion[0] = q0;
    gimbal_imu.quaternion[1] = q1;
    gimbal_imu.quaternion[2] = q2;
    gimbal_imu.quaternion[3] = q3;
    gimbal_imu.roll_deg = atan2f(2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)) * IMU_RAD_TO_DEG;
    gimbal_imu.pitch_deg = asinf(fmaxf(-1.0f, fminf(1.0f,
        2.0f * (q0*q2 - q3*q1)))) * IMU_RAD_TO_DEG;
    gimbal_imu.yaw_deg = atan2f(2.0f * (q0*q3 + q1*q2),
        1.0f - 2.0f * (q2*q2 + q3*q3)) * IMU_RAD_TO_DEG;
    yaw_delta = gimbal_imu.yaw_deg - yaw_last_deg;
    if (yaw_delta > 180.0f) { yaw_rounds--; }
    else if (yaw_delta < -180.0f) { yaw_rounds++; }
    gimbal_imu.yaw_total_deg = gimbal_imu.yaw_deg +
                               360.0f * (float)yaw_rounds;
    yaw_last_deg = gimbal_imu.yaw_deg;
}

HAL_StatusTypeDef GimbalImu_Init(void)
{
    float gyro[3];
    float accel[3];
    float temperature;
    float bias_sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t index;
    uint8_t error;

    memset((void *)&gimbal_imu, 0, sizeof(gimbal_imu));
    memset(gyro_bias, 0, sizeof(gyro_bias));
    memset(integral_feedback, 0, sizeof(integral_feedback));
    gimbal_imu.quaternion[0] = 1.0f;
    yaw_last_deg = 0.0f;
    yaw_rounds = 0;

    /* SPI1 和相关 GPIO 已由 CubeMX 在 MX_SPI1_Init/MX_GPIO_Init 中配置。 */
    if ((hspi1.Instance != SPI1) ||
        (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET))
    {
        gimbal_imu.init_error = 0x84U;
        return HAL_ERROR;
    }

    error = bmi088_init();
    gimbal_imu.init_error = error;
    if (error != 0U) { return HAL_ERROR; }

    /* 上电时保持云台静止，标定陀螺仪零偏。 */
    for (index = 0U; index < GIMBAL_IMU_CALIBRATION_SAMPLES; index++)
    {
        if (!imu_read_sensor(gyro, accel, &temperature))
        {
            gimbal_imu.init_error = 0x83U;
            return HAL_ERROR;
        }
        /* 传感器到车体坐标绕 Z 轴 180°。 */
        bias_sum[0] -= gyro[0];
        bias_sum[1] -= gyro[1];
        bias_sum[2] += gyro[2];
        HAL_Delay(1U);
    }
    gyro_bias[0] = bias_sum[0] / (float)GIMBAL_IMU_CALIBRATION_SAMPLES;
    gyro_bias[1] = bias_sum[1] / (float)GIMBAL_IMU_CALIBRATION_SAMPLES;
    gyro_bias[2] = bias_sum[2] / (float)GIMBAL_IMU_CALIBRATION_SAMPLES;
    gimbal_imu.calibrated = true;
    gimbal_imu.online = true;
    last_update_ms = HAL_GetTick();
    return HAL_OK;
}

bool GimbalImu_Update(void)
{
    float gyro[3];
    float accel[3];
    float temperature;
    float dt;
    uint32_t now_ms;

    if (!imu_read_sensor(gyro, accel, &temperature))
    {
        gimbal_imu.online = false;
        return false;
    }

    /* 绕 Z 轴 180° 的坐标变换：X、Y 取反，Z 不变。 */
    gyro[0] = -gyro[0] - gyro_bias[0];
    gyro[1] = -gyro[1] - gyro_bias[1];
    gyro[2] =  gyro[2] - gyro_bias[2];
    accel[0] = -accel[0];
    accel[1] = -accel[1];

    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_update_ms) * 0.001f;
    last_update_ms = now_ms;
    if (dt <= 0.0f || dt > 0.02f) { dt = GIMBAL_IMU_UPDATE_PERIOD_S; }

    memcpy((void *)gimbal_imu.gyro_rad_s, gyro, sizeof(gyro));
    memcpy((void *)gimbal_imu.accel_m_s2, accel, sizeof(accel));
    gimbal_imu.temperature_c = temperature;
    gimbal_imu.yaw_rate_deg_s += GIMBAL_IMU_YAW_RATE_FILTER_ALPHA *
        (gyro[2] * IMU_RAD_TO_DEG - gimbal_imu.yaw_rate_deg_s);
    imu_update_attitude(gyro[0], gyro[1], gyro[2],
                        accel[0], accel[1], accel[2], dt);
    gimbal_imu.update_count++;
    gimbal_imu.online = true;
    return true;
}

bool GimbalImu_Get(GimbalImu_Data_t *data)
{
    uint32_t primask;

    if (data == NULL) { return false; }
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&gimbal_imu, sizeof(*data));
    __set_PRIMASK(primask);
    return data->online && data->calibrated;
}
