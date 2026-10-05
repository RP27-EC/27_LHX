#include "imu.h"
#include "peripheral_config.h"
#include <math.h>
#include <string.h>

// 下板 BMI088 使用 SPI2。
#define IMU_ACCEL_CS_PORT              GPIOC
#define IMU_ACCEL_CS_PIN               GPIO_PIN_0
#define IMU_GYRO_CS_PORT               GPIOC
#define IMU_GYRO_CS_PIN                GPIO_PIN_3
#define IMU_SPI_MOSI_PIN               GPIO_PIN_1
#define IMU_SPI_MISO_PIN               GPIO_PIN_2
#define IMU_SPI_SCK_PORT               GPIOB
#define IMU_SPI_SCK_PIN                GPIO_PIN_13

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
#define IMU_GRAVITY                    9.80665f

static SPI_HandleTypeDef imu_spi; // 下板 BMI088 使用的独立 SPI 句柄。
static float gyro_bias[3]; // 标定完成后的三轴陀螺仪零偏。
static float gyro_bias_sum[3]; // 启动标定期间三轴零偏采样累加值。
static QuaternionEkf attitude_filter; // 四元数、在线零偏及固定矩阵工作区。
static uint32_t calibration_count; // 已累计的陀螺仪零偏标定样本数。
static uint32_t last_update_ms; // 上一次姿态更新的毫秒时间戳。
static float yaw_last_deg; // 上周期单圈 Yaw 角，用于跨圈判断。
static int32_t yaw_rounds; // Yaw 跨越正负 180 度的累计圈数。

volatile ChassisImu_Data_t chassis_imu; // 底盘 IMU 数据。

static void imu_delay_us(uint32_t us)
{
    uint32_t start, ticks;
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
    (void)HAL_SPI_TransmitReceive(&imu_spi, &value, &result, 1U, 10U);
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
    { data[index] = imu_spi_byte(0x55U); }
    imu_select(port, pin, false);
}

static bool imu_verify(GPIO_TypeDef *port, uint16_t pin,
                       uint8_t reg, uint8_t value, bool accel)
{
    imu_write(port, pin, reg, value);
    return imu_read_reg(port, pin, reg, accel) == value;
}

// 复位 BMI088 加速度计和陀螺仪，校验芯片 ID 并设置量程、滤波和采样参数。
static uint8_t bmi088_init(void)
{
    imu_select(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN, false);
    imu_select(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, false);
    HAL_Delay(10U);

    // 加速度计 SPI 模式需要先读两次 ID，再软复位。
    (void)imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                       BMI088_ACC_CHIP_ID, true);
    if (imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                     BMI088_ACC_CHIP_ID, true) != BMI088_ACC_CHIP_ID_VALUE)
    { return 0x80U; }
    imu_write(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
              BMI088_ACC_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    (void)imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                       BMI088_ACC_CHIP_ID, true);
    if (imu_read_reg(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                     BMI088_ACC_CHIP_ID, true) != BMI088_ACC_CHIP_ID_VALUE)
    { return 0x81U; }
    if (!imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_PWR_CTRL, 0x04U, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_PWR_CONF, 0x00U, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_CONF, 0xABU, true) ||
        !imu_verify(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                    BMI088_ACC_RANGE, 0x00U, true))
    { return 0x82U; }

    if (imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                     BMI088_GYRO_CHIP_ID, false) != BMI088_GYRO_CHIP_ID_VALUE)
    { return 0x40U; }
    imu_write(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
              BMI088_GYRO_SOFTRESET, 0xB6U);
    HAL_Delay(80U);
    if (imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                     BMI088_GYRO_CHIP_ID, false) != BMI088_GYRO_CHIP_ID_VALUE)
    { return 0x41U; }
    if (!imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_RANGE, 0x00U, false) ||
        !imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_BANDWIDTH, 0x82U, false) ||
        !imu_verify(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                    BMI088_GYRO_LPM1, 0x00U, false))
    { return 0x42U; }
    return 0U;
}

static bool imu_read_sensor(float gyro[3], float accel[3], float *temperature)
{
    uint8_t data[8];
    int16_t raw;
    uint8_t gyro_id;

    imu_read_burst(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN,
                   BMI088_ACC_X_L, true, data, 6U);
    raw = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    accel[0] = (float)raw * BMI088_ACC_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    accel[1] = (float)raw * BMI088_ACC_SENSITIVITY;
    raw = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    accel[2] = (float)raw * BMI088_ACC_SENSITIVITY;

    gyro_id = imu_read_reg(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN,
                           BMI088_GYRO_CHIP_ID, false);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) { return false; }
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

static bool imu_update_attitude(const float gyro[3], const float accel[3], float dt)
{
    QuaternionEkfConfig config = imu_config.attitude_ekf;
    float q0, q1, q2, q3;
    float yaw_delta;

    if (!quaternion_ekf_algorithm.ops.update(&attitude_filter, &config, gyro, accel, dt))
    { return false; }
    memcpy((void *)chassis_imu.quaternion, attitude_filter.q, sizeof(attitude_filter.q));
    memcpy((void *)chassis_imu.gyro_rad_s, attitude_filter.gyro, sizeof(attitude_filter.gyro));
    memcpy((void *)chassis_imu.ekf_bias_rad_s, attitude_filter.bias, sizeof(attitude_filter.bias));
    chassis_imu.ekf_chi_square = attitude_filter.chi_square;
    chassis_imu.ekf_accel_used = attitude_filter.accel_used;
    q0=attitude_filter.q[0]; q1=attitude_filter.q[1];
    q2=attitude_filter.q[2]; q3=attitude_filter.q[3];
    chassis_imu.roll_deg = atan2f(2.0f*(q0*q1 + q2*q3),
                                  1.0f - 2.0f*(q1*q1 + q2*q2)) * IMU_RAD_TO_DEG;
    chassis_imu.pitch_deg = asinf(fmaxf(-1.0f, fminf(1.0f,
                                   2.0f*(q0*q2 - q3*q1)))) * IMU_RAD_TO_DEG;
    chassis_imu.yaw_deg = atan2f(2.0f*(q0*q3 + q1*q2),
                                 1.0f - 2.0f*(q2*q2 + q3*q3)) * IMU_RAD_TO_DEG;
    yaw_delta = chassis_imu.yaw_deg - yaw_last_deg;
    if (yaw_delta > 180.0f) { yaw_rounds--; }
    else if (yaw_delta < -180.0f) { yaw_rounds++; }
    chassis_imu.yaw_total_deg = chassis_imu.yaw_deg + 360.0f*(float)yaw_rounds;
    yaw_last_deg = chassis_imu.yaw_deg;

    // 将机体系加速度旋转到世界系，并去除重力。
    chassis_imu.linear_accel_world_m_s2[0] =
        (1.0f-2.0f*(q2*q2+q3*q3))*chassis_imu.accel_m_s2[0] +
        2.0f*(q1*q2-q0*q3)*chassis_imu.accel_m_s2[1] +
        2.0f*(q1*q3+q0*q2)*chassis_imu.accel_m_s2[2];
    chassis_imu.linear_accel_world_m_s2[1] =
        2.0f*(q1*q2+q0*q3)*chassis_imu.accel_m_s2[0] +
        (1.0f-2.0f*(q1*q1+q3*q3))*chassis_imu.accel_m_s2[1] +
        2.0f*(q2*q3-q0*q1)*chassis_imu.accel_m_s2[2];
    chassis_imu.linear_accel_world_m_s2[2] =
        2.0f*(q1*q3-q0*q2)*chassis_imu.accel_m_s2[0] +
        2.0f*(q2*q3+q0*q1)*chassis_imu.accel_m_s2[1] +
        (1.0f-2.0f*(q1*q1+q2*q2))*chassis_imu.accel_m_s2[2] - IMU_GRAVITY;
    return true;
}

// 复位底盘姿态状态，配置 SPI 与片选，初始化 BMI088 并完成启动静止零偏标定。
HAL_StatusTypeDef ChassisImu_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    float gyro[3], accel[3], temperature;
    uint32_t index;
    uint8_t error;

    memset((void *)&chassis_imu, 0, sizeof(chassis_imu));
    chassis_imu.quaternion[0] = 1.0f;
    memset(gyro_bias, 0, sizeof(gyro_bias));
    memset(gyro_bias_sum, 0, sizeof(gyro_bias_sum));
    quaternion_ekf_algorithm.ops.init(&attitude_filter, NULL);
    calibration_count = 0U; last_update_ms = HAL_GetTick();
    yaw_last_deg = 0.0f; yaw_rounds = 0;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    clock.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) { return HAL_ERROR; }

    HAL_GPIO_WritePin(GPIOC, IMU_ACCEL_CS_PIN | IMU_GYRO_CS_PIN, GPIO_PIN_SET);
    gpio.Pin = IMU_ACCEL_CS_PIN | IMU_GYRO_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = IMU_SPI_MOSI_PIN | IMU_SPI_MISO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = IMU_SPI_SCK_PIN;
    HAL_GPIO_Init(IMU_SPI_SCK_PORT, &gpio);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

    imu_spi.Instance = SPI2;
    imu_spi.Init.Mode = SPI_MODE_MASTER;
    imu_spi.Init.Direction = SPI_DIRECTION_2LINES;
    imu_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    imu_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    imu_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    imu_spi.Init.NSS = SPI_NSS_SOFT;
    imu_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    imu_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    imu_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    imu_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    imu_spi.Init.CRCPolynomial = 0U;
    imu_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    imu_spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    imu_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    imu_spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    imu_spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    imu_spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    imu_spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    imu_spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    imu_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    imu_spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&imu_spi) != HAL_OK) { return HAL_ERROR; }
    error = bmi088_init();
    chassis_imu.init_error = error;
    if (error != 0U) { return HAL_ERROR; }

    // 主任务启动前保持底盘静止，完成三轴零偏标定，避免运动污染零偏。
    for (index = 0U; index < imu_config.gyro_calibration_samples; index++)
    {
        if (!imu_read_sensor(gyro, accel, &temperature))
        {
            chassis_imu.init_error = 0x83U;
            return HAL_ERROR;
        }
        gyro_bias_sum[0] += gyro[0] * imu_config.gyro_x_sign;
        gyro_bias_sum[1] += gyro[1] * imu_config.gyro_y_sign;
        gyro_bias_sum[2] += gyro[2] * imu_config.gyro_z_sign;
        HAL_Delay(1U);
    }
    calibration_count = imu_config.gyro_calibration_samples;
    gyro_bias[0] = gyro_bias_sum[0] / (float)calibration_count;
    gyro_bias[1] = gyro_bias_sum[1] / (float)calibration_count;
    gyro_bias[2] = gyro_bias_sum[2] / (float)calibration_count;
    chassis_imu.calibrated = true;
    chassis_imu.online = true;
    last_update_ms = HAL_GetTick();
    return HAL_OK;
}

bool ChassisImu_Update(void)
{
    float gyro[3], accel[3], temperature;
    float dt;
    uint32_t now_ms;

    if (!imu_read_sensor(gyro, accel, &temperature))
    { chassis_imu.online = false; return false; }
    // 传感器到车体坐标绕 Z 轴旋转 180 度。
    gyro[0] *= imu_config.gyro_x_sign; gyro[1] *= imu_config.gyro_y_sign;
    gyro[2] *= imu_config.gyro_z_sign;
    accel[0] *= imu_config.accel_x_sign; accel[1] *= imu_config.accel_y_sign;
    accel[2] *= imu_config.accel_z_sign;
    gyro[0] -= gyro_bias[0]; gyro[1] -= gyro_bias[1];
    gyro[2] -= gyro_bias[2];
    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_update_ms) * 0.001f;
    last_update_ms = now_ms;
    if (dt <= 0.0f || dt > 0.02f) { dt = imu_config.update_period_s; }
    memcpy((void *)chassis_imu.accel_m_s2, accel, sizeof(accel));
    chassis_imu.temperature_c = temperature;
    if (!imu_update_attitude(gyro, accel, dt))
    {
        chassis_imu.online = false;
        return false;
    }
    chassis_imu.yaw_rate_deg_s += imu_config.yaw_rate_filter_alpha *
        (chassis_imu.gyro_rad_s[2] * IMU_RAD_TO_DEG - chassis_imu.yaw_rate_deg_s);
    chassis_imu.update_count++;
    chassis_imu.online = true;
    return true;
}

bool ChassisImu_Get(ChassisImu_Data_t *data)
{
    uint32_t primask;
    if (data == NULL) { return false; }
    primask = __get_PRIMASK(); __disable_irq();
    memcpy(data, (const void *)&chassis_imu, sizeof(*data));
    __set_PRIMASK(primask);
    return data->online && data->calibrated;
}

bool ChassisImu_GetYawRate(float *yaw_rate_deg_s)
{
    ChassisImu_Data_t data;
    if (yaw_rate_deg_s == NULL || !ChassisImu_Get(&data)) { return false; }
    *yaw_rate_deg_s = data.yaw_rate_deg_s;
    return true;
}

// 绑定现有状态与函数，供外部通过模块结构体访问。
const ChassisImuModule chassis_imu_driver =
{
    .config = &imu_config,
    .data = {
        .sample = &chassis_imu,
    },
    .init = ChassisImu_Init,
    .update = ChassisImu_Update,
    .get = ChassisImu_Get,
    .get_yaw_rate = ChassisImu_GetYawRate,
};
