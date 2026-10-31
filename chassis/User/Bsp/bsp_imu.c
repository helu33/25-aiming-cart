#include "bsp_imu.h"

#include "board_config.h"
#include "i2c.h"
#include "iwdg.h"
#include <stddef.h>

#define ICM40607_REG_DEVICE_CONFIG  (0x11U)
#define ICM40607_REG_DRIVE_CONFIG   (0x13U)
#define ICM40607_REG_TEMP_DATA1     (0x1DU)
#define ICM40607_REG_PWR_MGMT0      (0x4EU)
#define ICM40607_REG_GYRO_CONFIG0   (0x4FU)
#define ICM40607_REG_ACCEL_CONFIG0  (0x50U)
#define ICM40607_REG_WHO_AM_I       (0x75U)

#define ICM40607_WHO_AM_I_VALUE     (0x38U)
#define ICM40607_DEVICE_RESET       (0x01U)
#define ICM40607_I2C_SLEW_RATE      (0x08U)
#define ICM40607_GYRO_500DPS_100HZ  (0x48U)
#define ICM40607_ACCEL_4G_100HZ     (0x48U)
#define ICM40607_SIX_AXIS_LN        (0x0FU)

#define ICM40607_I2C_TIMEOUT_MS     (20U)
#define ICM40607_GRAVITY_M_S2       (9.80665f)
#define ICM40607_ACCEL_LSB_PER_G    (8192.0f)
#define ICM40607_GYRO_LSB_PER_DPS   (65.536f)
#define ICM40607_DEG_TO_RAD         (0.01745329251994329577f)

static uint8_t imu_ready;
static uint8_t who_am_i;
static float accel_bias_m_s2[3];
static float gyro_bias_rad_s[3];

static uint16_t i2c_address(void)
{
    return (uint16_t)(BOARD_IMU_I2C_ADDRESS_7BIT << 1U);
}

static uint8_t reg_write(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c2, i2c_address(), reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, ICM40607_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t reg_read(uint8_t reg, uint8_t *data, uint16_t length)
{
    return (HAL_I2C_Mem_Read(&hi2c2, i2c_address(), reg, I2C_MEMADD_SIZE_8BIT,
                             data, length, ICM40607_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}


static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}


static float axis_select(const float axes[3], uint8_t source)
{
    return (source < 3U) ? axes[source] : 0.0f;
}

static int16_t axis_select_raw(const int16_t axes[3], uint8_t source)
{
    return (source < 3U) ? axes[source] : 0;
}


static uint8_t read_uncalibrated(imu_sample_t *sample)
{
    uint8_t data[14];
    int16_t accel_sensor[3];
    int16_t gyro_sensor[3];
    float accel_sensor_m_s2[3];
    float gyro_sensor_rad_s[3];
    uint8_t i;

    if ((sample == NULL) || (imu_ready == 0U)) return 0U;
    if (reg_read(ICM40607_REG_TEMP_DATA1, data, (uint16_t)sizeof(data)) == 0U) return 0U;

    sample->temperature_raw = be_i16(&data[0]);
    for (i = 0U; i < 3U; i++) {
        accel_sensor[i] = be_i16(&data[2U + (2U * i)]);
        gyro_sensor[i] = be_i16(&data[8U + (2U * i)]);
        accel_sensor_m_s2[i] = ((float)accel_sensor[i] / ICM40607_ACCEL_LSB_PER_G) * ICM40607_GRAVITY_M_S2;
        gyro_sensor_rad_s[i] = ((float)gyro_sensor[i] / ICM40607_GYRO_LSB_PER_DPS) * ICM40607_DEG_TO_RAD;
    }

    sample->accel_raw[0] = axis_select_raw(accel_sensor, BOARD_IMU_AXIS_X_SOURCE);
    sample->accel_raw[1] = axis_select_raw(accel_sensor, BOARD_IMU_AXIS_Y_SOURCE);
    sample->accel_raw[2] = axis_select_raw(accel_sensor, BOARD_IMU_AXIS_Z_SOURCE);
    sample->gyro_raw[0] = axis_select_raw(gyro_sensor, BOARD_IMU_AXIS_X_SOURCE);
    sample->gyro_raw[1] = axis_select_raw(gyro_sensor, BOARD_IMU_AXIS_Y_SOURCE);
    sample->gyro_raw[2] = axis_select_raw(gyro_sensor, BOARD_IMU_AXIS_Z_SOURCE);

    sample->accel_m_s2[0] = BOARD_IMU_AXIS_X_SIGN * axis_select(accel_sensor_m_s2, BOARD_IMU_AXIS_X_SOURCE);
    sample->accel_m_s2[1] = BOARD_IMU_AXIS_Y_SIGN * axis_select(accel_sensor_m_s2, BOARD_IMU_AXIS_Y_SOURCE);
    sample->accel_m_s2[2] = BOARD_IMU_AXIS_Z_SIGN * axis_select(accel_sensor_m_s2, BOARD_IMU_AXIS_Z_SOURCE);
    sample->gyro_rad_s[0] = BOARD_IMU_AXIS_X_SIGN * axis_select(gyro_sensor_rad_s, BOARD_IMU_AXIS_X_SOURCE);
    sample->gyro_rad_s[1] = BOARD_IMU_AXIS_Y_SIGN * axis_select(gyro_sensor_rad_s, BOARD_IMU_AXIS_Y_SOURCE);
    sample->gyro_rad_s[2] = BOARD_IMU_AXIS_Z_SIGN * axis_select(gyro_sensor_rad_s, BOARD_IMU_AXIS_Z_SOURCE);
    sample->temperature_c = ((float)sample->temperature_raw / 132.48f) + 25.0f;
    return 1U;
}

uint8_t imu_bsp_init(void)
{
    uint8_t i;
    imu_ready = 0U;
    who_am_i = 0U;
    for (i = 0U; i < 3U; i++) {
        accel_bias_m_s2[i] = 0.0f;
        gyro_bias_rad_s[i] = 0.0f;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c2, i2c_address(), 3U, ICM40607_I2C_TIMEOUT_MS) != HAL_OK) return 0U;
    if (reg_write(ICM40607_REG_DEVICE_CONFIG, ICM40607_DEVICE_RESET) == 0U) return 0U;
    HAL_Delay(10U);
    if (reg_read(ICM40607_REG_WHO_AM_I, &who_am_i, 1U) == 0U) return 0U;
    if (who_am_i != ICM40607_WHO_AM_I_VALUE) return 0U;
    if (reg_write(ICM40607_REG_DRIVE_CONFIG, ICM40607_I2C_SLEW_RATE) == 0U) return 0U;
    if (reg_write(ICM40607_REG_GYRO_CONFIG0, ICM40607_GYRO_500DPS_100HZ) == 0U) return 0U;
    if (reg_write(ICM40607_REG_ACCEL_CONFIG0, ICM40607_ACCEL_4G_100HZ) == 0U) return 0U;
    if (reg_write(ICM40607_REG_PWR_MGMT0, ICM40607_SIX_AXIS_LN) == 0U) return 0U;
    HAL_Delay(50U);
    imu_ready = 1U;
    return 1U;
}

//初始化IMU传感器，读取原始数据并进行校准。
uint8_t imu_bsp_calibrate(uint16_t sample_count, uint16_t sample_delay_ms)
{
    imu_sample_t sample;
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t sample_index;
    uint8_t axis;

    if ((imu_ready == 0U) || (sample_count == 0U)) return 0U;
    for (axis = 0U; axis < 3U; axis++) {
        accel_bias_m_s2[axis] = 0.0f;
        gyro_bias_rad_s[axis] = 0.0f;
    }

    for (sample_index = 0U; sample_index < sample_count; sample_index++) {
        if (read_uncalibrated(&sample) == 0U) return 0U;
        for (axis = 0U; axis < 3U; axis++) {
            accel_sum[axis] += sample.accel_m_s2[axis];
            gyro_sum[axis] += sample.gyro_rad_s[axis];
        }
        /* Calibration is intentionally blocking; keep the safety watchdog alive. */
        //HAL_IWDG_Refresh(看门狗)
        (void)HAL_IWDG_Refresh(&hiwdg);
        //延时检测数据，作为周期
        HAL_Delay(sample_delay_ms);
    }
    //取平均数作为最终结果

    for (axis = 0U; axis < 3U; axis++) {
        accel_bias_m_s2[axis] = accel_sum[axis] / (float)sample_count;
        gyro_bias_rad_s[axis] = gyro_sum[axis] / (float)sample_count;
    }
    /* Default installation assumes vehicle +Z points upward while stationary. */
    
    //z轴还需减去重力加速度
    accel_bias_m_s2[2] -= ICM40607_GRAVITY_M_S2;
    return 1U;
}

//读取IMU传感器数据，返回校准后的加速度、角速度和温度数据。
uint8_t imu_bsp_read(imu_sample_t *sample)
{
    uint8_t axis;
    if (read_uncalibrated(sample) == 0U) return 0U;
    for (axis = 0U; axis < 3U; axis++) {
        //减去误差估计值
        sample->accel_m_s2[axis] -= accel_bias_m_s2[axis];
        sample->gyro_rad_s[axis] -= gyro_bias_rad_s[axis];
    }
    return 1U;
}

uint8_t imu_bsp_ready_get(void) { return imu_ready; }
uint8_t imu_bsp_who_am_i_get(void) { return who_am_i; }
