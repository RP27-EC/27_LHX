#include "imu.h"
#include "peripheral_config.h"
#include "spi.h"
#include <math.h>
#include <string.h>

// 上板 BMI088：SPI1，PA4 加速度计 CS，PB0 陀螺仪 CS。
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
#define IMU_SPI_TIMEOUT_MS             2U
#define IMU_BURST_MAX_BYTES            6U

static float gyro_bias[3]; // 标定得到的三轴陀螺仪零偏。
static QuaternionEkf attitude_filter; // 四元数、在线零偏及固定矩阵工作区。
static uint32_t last_update_ms; // 上一次姿态更新的毫秒时间戳。
static uint32_t last_success_ms; // 上一次完整读取 BMI088 的时间。
static float yaw_last_deg; // 上一周期单圈 Yaw 角，用于跨圈判断。
static int32_t yaw_rounds; // Yaw 跨越正负 180 度的累计圈数。

volatile GimbalImu_Data_t gimbal_imu; // 云台 IMU 数据。

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

static void imu_select(GPIO_TypeDef *port, uint16_t pin, bool selected)
{
    HAL_GPIO_WritePin(port, pin, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool imu_transfer(GPIO_TypeDef *port, uint16_t pin,
                         uint8_t *tx, uint8_t *rx, uint16_t size)
{
    HAL_StatusTypeDef status;

    imu_select(port, pin, true);
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, size,
                                     IMU_SPI_TIMEOUT_MS);
    imu_select(port, pin, false);
    return status == HAL_OK;
}

static bool imu_write(GPIO_TypeDef *port, uint16_t pin,
                      uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2];
    bool ok = imu_transfer(port, pin, tx, rx, sizeof(tx));

    imu_delay_us(150U);
    return ok;
}

static uint8_t imu_read_reg(GPIO_TypeDef *port, uint16_t pin,
                            uint8_t reg, bool accel)
{
    uint8_t tx[3] = {0U, 0x55U, 0x55U};
    uint8_t rx[3];
    uint16_t size = accel ? 3U : 2U;

    tx[0] = reg | 0x80U;
    if (!imu_transfer(port, pin, tx, rx, size)) { return 0xFFU; }
    return rx[size - 1U];
}

static bool imu_read_burst(GPIO_TypeDef *port, uint16_t pin, uint8_t reg,
                           bool accel, uint8_t *data, uint32_t length)
{
    uint8_t tx[IMU_BURST_MAX_BYTES + 2U];
    uint8_t rx[IMU_BURST_MAX_BYTES + 2U];
    uint32_t prefix = accel ? 2U : 1U;
    uint32_t size = length + prefix;

    if (data == NULL || length == 0U || length > IMU_BURST_MAX_BYTES)
    { return false; }
    memset(tx, 0x55, size);
    tx[0] = reg | 0x80U;
    if (!imu_transfer(port, pin, tx, rx, (uint16_t)size)) { return false; }
    memcpy(data, &rx[prefix], length);
    return true;
}

static bool imu_verify(GPIO_TypeDef *port, uint16_t pin,
                       uint8_t reg, uint8_t value, bool accel)
{
    return imu_write(port, pin, reg, value) &&
           imu_read_reg(port, pin, reg, accel) == value;
}

// 复位 BMI088 加速度计和陀螺仪，校验芯片 ID 并设置量程、滤波和采样参数。
static uint8_t bmi088_init(void)
{
    imu_select(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN, false);
    imu_select(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, false);
    HAL_Delay(10U);

    // BMI088 加速度计进入 SPI 模式时需要连续读取两次。
    (void)imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                       BMI088_ACC_CHIP_ID, true);
    if (imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                     BMI088_ACC_CHIP_ID, true) != BMI088_ACC_CHIP_ID_VALUE)
    {
        return 0x80U;
    }
    if (!imu_write(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                   BMI088_ACC_SOFTRESET, 0xB6U)) { return 0x81U; }
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
    if (!imu_write(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                   BMI088_GYRO_SOFTRESET, 0xB6U)) { return 0x41U; }
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

    if (!imu_read_burst(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                        BMI088_ACC_X_L, true, data, 6U)) { return false; }
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
    if (!imu_read_burst(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                        BMI088_GYRO_X_L, false, data, 6U)) { return false; }
    raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    gyro[0] = (float)raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    gyro[1] = (float)raw * BMI088_GYRO_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    gyro[2] = (float)raw * BMI088_GYRO_SENSITIVITY;

    if (!imu_read_burst(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                        BMI088_ACC_TEMP_M, true, data, 2U)) { return false; }
    raw = (int16_t)(((uint16_t)data[0] << 3) | (data[1] >> 5));
    if (raw > 1023) { raw -= 2048; }
    *temperature = (float)raw * 0.125f + 23.0f;
    return true;
}

static bool imu_update_attitude(const float gyro[3], const float accel[3], float dt)
{
    QuaternionEkfConfig config = gimbal_imu_config.attitude_ekf;
    float q0, q1, q2, q3;
    float yaw_delta;

    if (!quaternion_ekf_algorithm.ops.update(&attitude_filter, &config, gyro, accel, dt))
    { return false; }
    memcpy((void *)gimbal_imu.quaternion, attitude_filter.q, sizeof(attitude_filter.q));
    memcpy((void *)gimbal_imu.gyro_rad_s, attitude_filter.gyro, sizeof(attitude_filter.gyro));
    memcpy((void *)gimbal_imu.ekf_bias_rad_s, attitude_filter.bias, sizeof(attitude_filter.bias));
    gimbal_imu.ekf_chi_square = attitude_filter.chi_square;
    gimbal_imu.ekf_accel_used = attitude_filter.accel_used;
    q0=attitude_filter.q[0]; q1=attitude_filter.q[1];
    q2=attitude_filter.q[2]; q3=attitude_filter.q[3];
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
    return true;
}

// 复位云台姿态状态，初始化 BMI088，静止采样陀螺零偏并建立 EKF 初始姿态。
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
    quaternion_ekf_algorithm.ops.init(&attitude_filter, NULL);
    gimbal_imu.quaternion[0] = 1.0f;
    yaw_last_deg = 0.0f;
    yaw_rounds = 0;
    last_success_ms = 0U;

    // SPI1 和相关 GPIO 已由 CubeMX 在 MX_SPI1_Init/MX_GPIO_Init 中配置。
    if ((hspi1.Instance != SPI1) ||
        (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET))
    {
        gimbal_imu.init_error = 0x84U;
        return HAL_ERROR;
    }

    error = bmi088_init();
    gimbal_imu.init_error = error;
    if (error != 0U) { return HAL_ERROR; }

    // 上电时保持云台静止，标定陀螺仪零偏。
    for (index = 0U; index < gimbal_imu_config.calibration_samples; index++)
    {
        if (!imu_read_sensor(gyro, accel, &temperature))
        {
            gimbal_imu.init_error = 0x83U;
            return HAL_ERROR;
        }
        // 传感器到车体坐标绕 Z 轴 180°。
        bias_sum[0] -= gyro[0];
        bias_sum[1] -= gyro[1];
        bias_sum[2] += gyro[2];
        HAL_Delay(1U);
    }
    gyro_bias[0] = bias_sum[0] / (float)gimbal_imu_config.calibration_samples;
    gyro_bias[1] = bias_sum[1] / (float)gimbal_imu_config.calibration_samples;
    gyro_bias[2] = bias_sum[2] / (float)gimbal_imu_config.calibration_samples;
    gimbal_imu.calibrated = true;
    gimbal_imu.online = true;
    last_update_ms = HAL_GetTick();
    last_success_ms = last_update_ms;
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
        if ((uint32_t)(HAL_GetTick() - last_success_ms) >=
            gimbal_imu_config.read_timeout_ms)
        { gimbal_imu.online = false; }
        return false;
    }

    // 绕 Z 轴 180° 的坐标变换：X、Y 取反，Z 不变。
    gyro[0] = -gyro[0] - gyro_bias[0];
    gyro[1] = -gyro[1] - gyro_bias[1];
    gyro[2] =  gyro[2] - gyro_bias[2];
    accel[0] = -accel[0];
    accel[1] = -accel[1];

    now_ms = HAL_GetTick();
    last_success_ms = now_ms;
    dt = (float)(uint32_t)(now_ms - last_update_ms) * 0.001f;
    last_update_ms = now_ms;
    if (dt <= 0.0f || dt > 0.02f) { dt = gimbal_imu_config.update_period_s; }

    memcpy((void *)gimbal_imu.accel_m_s2, accel, sizeof(accel));
    gimbal_imu.temperature_c = temperature;
    if (!imu_update_attitude(gyro, accel, dt))
    {
        gimbal_imu.online = false;
        return false;
    }
    gimbal_imu.yaw_rate_deg_s += gimbal_imu_config.yaw_rate_filter_alpha *
        (gimbal_imu.gyro_rad_s[2] * IMU_RAD_TO_DEG - gimbal_imu.yaw_rate_deg_s);
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

// 绑定现有状态与函数，供外部通过模块结构体访问。
const GimbalImuModule gimbal_imu_driver =
{
    .config = &gimbal_imu_config,
    .data = {
        .sample = &gimbal_imu,
    },
    .init = GimbalImu_Init,
    .update = GimbalImu_Update,
    .get = GimbalImu_Get,
};
