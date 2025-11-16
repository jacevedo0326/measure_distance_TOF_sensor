#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// MPU-6500 I2C Address (AD0 = 0)
#define MPU6500_I2C_ADDR            0x68
#define MPU6500_I2C_ADDR_ALT        0x69  // AD0 = 1

// MPU-6500 Register Map
#define MPU6500_SELF_TEST_X_GYRO    0x00
#define MPU6500_SELF_TEST_Y_GYRO    0x01
#define MPU6500_SELF_TEST_Z_GYRO    0x02
#define MPU6500_SELF_TEST_X_ACCEL   0x0D
#define MPU6500_SELF_TEST_Y_ACCEL   0x0E
#define MPU6500_SELF_TEST_Z_ACCEL   0x0F
#define MPU6500_SMPLRT_DIV          0x19
#define MPU6500_CONFIG              0x1A
#define MPU6500_GYRO_CONFIG         0x1B
#define MPU6500_ACCEL_CONFIG        0x1C
#define MPU6500_ACCEL_CONFIG2       0x1D
#define MPU6500_LP_ACCEL_ODR        0x1E
#define MPU6500_WOM_THR             0x1F
#define MPU6500_FIFO_EN             0x23
#define MPU6500_I2C_MST_CTRL        0x24
#define MPU6500_I2C_SLV0_ADDR       0x25
#define MPU6500_I2C_SLV0_REG        0x26
#define MPU6500_I2C_SLV0_CTRL       0x27
#define MPU6500_I2C_SLV1_ADDR       0x28
#define MPU6500_I2C_SLV1_REG        0x29
#define MPU6500_I2C_SLV1_CTRL       0x2A
#define MPU6500_I2C_SLV2_ADDR       0x2B
#define MPU6500_I2C_SLV2_REG        0x2C
#define MPU6500_I2C_SLV2_CTRL       0x2D
#define MPU6500_I2C_SLV3_ADDR       0x2E
#define MPU6500_I2C_SLV3_REG        0x2F
#define MPU6500_I2C_SLV3_CTRL       0x30
#define MPU6500_I2C_SLV4_ADDR       0x31
#define MPU6500_I2C_SLV4_REG        0x32
#define MPU6500_I2C_SLV4_DO         0x33
#define MPU6500_I2C_SLV4_CTRL       0x34
#define MPU6500_I2C_SLV4_DI         0x35
#define MPU6500_I2C_MST_STATUS      0x36
#define MPU6500_INT_PIN_CFG         0x37
#define MPU6500_INT_ENABLE          0x38
#define MPU6500_INT_STATUS          0x3A
#define MPU6500_ACCEL_XOUT_H        0x3B
#define MPU6500_ACCEL_XOUT_L        0x3C
#define MPU6500_ACCEL_YOUT_H        0x3D
#define MPU6500_ACCEL_YOUT_L        0x3E
#define MPU6500_ACCEL_ZOUT_H        0x3F
#define MPU6500_ACCEL_ZOUT_L        0x40
#define MPU6500_TEMP_OUT_H          0x41
#define MPU6500_TEMP_OUT_L          0x42
#define MPU6500_GYRO_XOUT_H         0x43
#define MPU6500_GYRO_XOUT_L         0x44
#define MPU6500_GYRO_YOUT_H         0x45
#define MPU6500_GYRO_YOUT_L         0x46
#define MPU6500_GYRO_ZOUT_H         0x47
#define MPU6500_GYRO_ZOUT_L         0x48
#define MPU6500_SIGNAL_PATH_RESET   0x68
#define MPU6500_MOT_DETECT_CTRL     0x69
#define MPU6500_USER_CTRL           0x6A
#define MPU6500_PWR_MGMT_1          0x6B
#define MPU6500_PWR_MGMT_2          0x6C
#define MPU6500_FIFO_COUNTH         0x72
#define MPU6500_FIFO_COUNTL         0x73
#define MPU6500_FIFO_R_W            0x74
#define MPU6500_WHO_AM_I            0x75

// WHO_AM_I Register Value
#define MPU6500_WHO_AM_I_VAL        0x70

// Gyroscope Full Scale Range
typedef enum {
    MPU6500_GYRO_FS_250DPS  = 0,  // ±250°/s
    MPU6500_GYRO_FS_500DPS  = 1,  // ±500°/s
    MPU6500_GYRO_FS_1000DPS = 2,  // ±1000°/s
    MPU6500_GYRO_FS_2000DPS = 3   // ±2000°/s
} mpu6500_gyro_fs_t;

// Accelerometer Full Scale Range
typedef enum {
    MPU6500_ACCEL_FS_2G  = 0,     // ±2g
    MPU6500_ACCEL_FS_4G  = 1,     // ±4g
    MPU6500_ACCEL_FS_8G  = 2,     // ±8g
    MPU6500_ACCEL_FS_16G = 3      // ±16g
} mpu6500_accel_fs_t;

// Digital Low Pass Filter (DLPF) Configuration
typedef enum {
    MPU6500_DLPF_250HZ  = 0,
    MPU6500_DLPF_184HZ  = 1,
    MPU6500_DLPF_92HZ   = 2,
    MPU6500_DLPF_41HZ   = 3,
    MPU6500_DLPF_20HZ   = 4,
    MPU6500_DLPF_10HZ   = 5,
    MPU6500_DLPF_5HZ    = 6
} mpu6500_dlpf_t;

// MPU-6500 Data Structures
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6500_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu6500_scaled_data_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t dev_addr;
    mpu6500_gyro_fs_t gyro_fs;
    mpu6500_accel_fs_t accel_fs;
    float gyro_scale;
    float accel_scale;
} mpu6500_t;

// Function Prototypes

/**
 * @brief Initialize MPU-6500
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param i2c_port I2C port number
 * @param dev_addr Device I2C address (MPU6500_I2C_ADDR or MPU6500_I2C_ADDR_ALT)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_init(mpu6500_t *mpu, i2c_port_t i2c_port, uint8_t dev_addr);

/**
 * @brief Test MPU-6500 connectivity by reading WHO_AM_I register
 *
 * @param mpu Pointer to MPU-6500 handle
 * @return esp_err_t ESP_OK if device responds correctly
 */
esp_err_t mpu6500_test_connection(mpu6500_t *mpu);

/**
 * @brief Configure gyroscope full scale range
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param gyro_fs Gyroscope full scale range
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_set_gyro_fs(mpu6500_t *mpu, mpu6500_gyro_fs_t gyro_fs);

/**
 * @brief Configure accelerometer full scale range
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param accel_fs Accelerometer full scale range
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_set_accel_fs(mpu6500_t *mpu, mpu6500_accel_fs_t accel_fs);

/**
 * @brief Configure digital low pass filter
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param dlpf DLPF configuration
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_set_dlpf(mpu6500_t *mpu, mpu6500_dlpf_t dlpf);

/**
 * @brief Read raw gyroscope data
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param data Pointer to raw data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_gyro_raw(mpu6500_t *mpu, mpu6500_raw_data_t *data);

/**
 * @brief Read scaled gyroscope data (in degrees/second)
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param data Pointer to scaled data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_gyro(mpu6500_t *mpu, mpu6500_scaled_data_t *data);

/**
 * @brief Read raw accelerometer data
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param data Pointer to raw data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_accel_raw(mpu6500_t *mpu, mpu6500_raw_data_t *data);

/**
 * @brief Read scaled accelerometer data (in g)
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param data Pointer to scaled data structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_accel(mpu6500_t *mpu, mpu6500_scaled_data_t *data);

/**
 * @brief Read temperature from sensor
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param temp Pointer to temperature value (in °C)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_temperature(mpu6500_t *mpu, float *temp);

/**
 * @brief Read all sensor data at once
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param accel Pointer to accelerometer scaled data structure
 * @param gyro Pointer to gyroscope scaled data structure
 * @param temp Pointer to temperature value (in °C)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_read_all(mpu6500_t *mpu, mpu6500_scaled_data_t *accel,
                           mpu6500_scaled_data_t *gyro, float *temp);

/**
 * @brief Reset MPU-6500
 *
 * @param mpu Pointer to MPU-6500 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_reset(mpu6500_t *mpu);

/**
 * @brief Put MPU-6500 into sleep mode
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param enable true to enable sleep mode, false to wake up
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6500_sleep(mpu6500_t *mpu, bool enable);

/**
 * @brief Calculate tilt angles from accelerometer (pitch and roll)
 *
 * This function calculates the tilt angles using the accelerometer data.
 * It's most accurate when the device is stationary (not experiencing acceleration).
 *
 * @param accel Pointer to accelerometer data (in g)
 * @param pitch Pointer to store pitch angle (rotation around Y axis, in degrees)
 * @param roll Pointer to store roll angle (rotation around X axis, in degrees)
 */
void mpu6500_get_accel_angles(mpu6500_scaled_data_t *accel, float *pitch, float *roll);

/**
 * @brief Calculate yaw angle change from gyroscope Z-axis
 *
 * Integrates gyroscope Z-axis to estimate yaw angle change.
 * Note: This will drift over time and needs periodic correction.
 *
 * @param gyro Pointer to gyroscope data (in °/s)
 * @param dt Time delta since last reading (in seconds)
 * @param yaw Pointer to current yaw angle (updated with new value)
 */
void mpu6500_integrate_gyro_yaw(mpu6500_scaled_data_t *gyro, float dt, float *yaw);

/**
 * @brief Calculate pedal angle and convert to percentage (ACCELEROMETER - not recommended in moving vehicle)
 *
 * Measures the angle of a pedal (brake/throttle) and converts to percentage.
 * WARNING: This is affected by vehicle acceleration/braking. Use gyro-based method instead.
 *
 * @param accel Pointer to accelerometer data (in g)
 * @param min_angle Minimum angle (pedal fully released, in degrees)
 * @param max_angle Maximum angle (pedal fully pressed, in degrees)
 * @param angle Pointer to store current pedal angle (in degrees)
 * @param percentage Pointer to store pedal position (0-100%)
 */
void mpu6500_get_pedal_position(mpu6500_scaled_data_t *accel,
                                float min_angle, float max_angle,
                                float *angle, float *percentage);

/**
 * @brief Update pedal angle using gyroscope integration (recommended for moving vehicle)
 *
 * Integrates gyroscope rotation rate to track pedal angle.
 * Not affected by vehicle acceleration/braking.
 * Requires periodic reset when pedal returns to rest position.
 *
 * @param gyro Pointer to gyroscope data (in °/s)
 * @param dt Time delta since last reading (in seconds)
 * @param current_angle Pointer to current pedal angle (updated with new value)
 * @param min_angle Minimum angle (pedal fully released, in degrees)
 * @param max_angle Maximum angle (pedal fully pressed, in degrees)
 * @param percentage Pointer to store pedal position (0-100%)
 */
void mpu6500_update_pedal_gyro(mpu6500_scaled_data_t *gyro, float dt,
                               float *current_angle,
                               float min_angle, float max_angle,
                               float *percentage);

/**
 * @brief Reset pedal angle to rest position
 *
 * Call this when pedal is known to be in rest position (fully released).
 * This prevents gyro drift accumulation.
 *
 * @param current_angle Pointer to current pedal angle (will be set to min_angle)
 * @param min_angle Minimum angle (pedal fully released, in degrees)
 */
void mpu6500_reset_pedal_angle(float *current_angle, float min_angle);

/**
 * @brief Display visual orientation of the MPU-6500 sensor (ACCELEROMETER ONLY)
 *
 * Shows an ASCII art representation of the sensor's 3D orientation
 * based on accelerometer data. Updates continuously to show real-time orientation.
 *
 * WARNING: Only accurate when stationary! Use for debugging/calibration only.
 * Use mpu6500_show_orientation_gyro() for moving vehicle applications.
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param duration_ms Duration to display animation (0 = run forever, Ctrl+C to stop)
 * @param update_rate_ms Update rate in milliseconds (default: 100ms = 10Hz)
 */
void mpu6500_show_orientation(mpu6500_t *mpu, uint32_t duration_ms, uint32_t update_rate_ms);

/**
 * @brief Display visual orientation using GYROSCOPE + ACCELEROMETER (for moving vehicles)
 *
 * Shows an ASCII art representation of the sensor's 3D orientation
 * using gyroscope integration with accelerometer correction (complementary filter).
 * This version works correctly even when the vehicle is accelerating/braking.
 *
 * @param mpu Pointer to MPU-6500 handle
 * @param duration_ms Duration to display animation (0 = run forever, Ctrl+C to stop)
 * @param update_rate_ms Update rate in milliseconds (default: 50ms = 20Hz)
 * @param alpha Complementary filter coefficient (0.95-0.99, default 0.98)
 */
void mpu6500_show_orientation_gyro(mpu6500_t *mpu, uint32_t duration_ms, uint32_t update_rate_ms, float alpha);

#endif // MPU6500_H
