#include "mpu6500.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MPU6500";

// I2C timeout
#define I2C_TIMEOUT_MS  1000

// Helper functions for I2C communication
static esp_err_t mpu6500_write_reg(mpu6500_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(mpu->i2c_port, mpu->dev_addr, write_buf, 2,
                                      I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6500_read_reg(mpu6500_t *mpu, uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(mpu->i2c_port, mpu->dev_addr,
                                        &reg, 1, data, 1,
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6500_read_regs(mpu6500_t *mpu, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(mpu->i2c_port, mpu->dev_addr,
                                        &reg, 1, data, len,
                                        I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Calculate scale factors based on full scale range
static void mpu6500_calculate_scales(mpu6500_t *mpu)
{
    // Gyroscope sensitivity (LSB/°/s)
    switch (mpu->gyro_fs) {
        case MPU6500_GYRO_FS_250DPS:
            mpu->gyro_scale = 131.0f;
            break;
        case MPU6500_GYRO_FS_500DPS:
            mpu->gyro_scale = 65.5f;
            break;
        case MPU6500_GYRO_FS_1000DPS:
            mpu->gyro_scale = 32.8f;
            break;
        case MPU6500_GYRO_FS_2000DPS:
            mpu->gyro_scale = 16.4f;
            break;
        default:
            mpu->gyro_scale = 131.0f;
    }

    // Accelerometer sensitivity (LSB/g)
    switch (mpu->accel_fs) {
        case MPU6500_ACCEL_FS_2G:
            mpu->accel_scale = 16384.0f;
            break;
        case MPU6500_ACCEL_FS_4G:
            mpu->accel_scale = 8192.0f;
            break;
        case MPU6500_ACCEL_FS_8G:
            mpu->accel_scale = 4096.0f;
            break;
        case MPU6500_ACCEL_FS_16G:
            mpu->accel_scale = 2048.0f;
            break;
        default:
            mpu->accel_scale = 16384.0f;
    }
}

esp_err_t mpu6500_init(mpu6500_t *mpu, i2c_port_t i2c_port, uint8_t dev_addr)
{
    esp_err_t ret;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu->i2c_port = i2c_port;
    mpu->dev_addr = dev_addr;

    ESP_LOGI(TAG, "Initializing MPU-6500...");

    // Test connection first
    ret = mpu6500_test_connection(mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to MPU-6500");
        return ret;
    }

    // Reset device
    ret = mpu6500_reset(mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU-6500");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete

    // Wake up from sleep mode
    ret = mpu6500_sleep(mpu, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU-6500");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Set clock source to auto-select best available
    ret = mpu6500_write_reg(mpu, MPU6500_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set clock source");
        return ret;
    }

    // Configure gyroscope (default ±250°/s)
    ret = mpu6500_set_gyro_fs(mpu, MPU6500_GYRO_FS_250DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }

    // Configure accelerometer (default ±2g)
    ret = mpu6500_set_accel_fs(mpu, MPU6500_ACCEL_FS_2G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }

    // Set DLPF to 92 Hz
    ret = mpu6500_set_dlpf(mpu, MPU6500_DLPF_92HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DLPF");
        return ret;
    }

    // Set sample rate divider (1kHz / (1 + 9) = 100Hz)
    ret = mpu6500_write_reg(mpu, MPU6500_SMPLRT_DIV, 9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate");
        return ret;
    }

    ESP_LOGI(TAG, "MPU-6500 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6500_test_connection(mpu6500_t *mpu)
{
    esp_err_t ret;
    uint8_t who_am_i;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_reg(mpu, MPU6500_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, MPU6500_WHO_AM_I_VAL);

    if (who_am_i != MPU6500_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch! Got 0x%02X, expected 0x%02X",
                 who_am_i, MPU6500_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU-6500 connection test passed");
    return ESP_OK;
}

esp_err_t mpu6500_set_gyro_fs(mpu6500_t *mpu, mpu6500_gyro_fs_t gyro_fs)
{
    esp_err_t ret;
    uint8_t config;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read current config
    ret = mpu6500_read_reg(mpu, MPU6500_GYRO_CONFIG, &config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Clear FS_SEL bits and set new value
    config = (config & ~0x18) | (gyro_fs << 3);

    ret = mpu6500_write_reg(mpu, MPU6500_GYRO_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }

    mpu->gyro_fs = gyro_fs;
    mpu6500_calculate_scales(mpu);

    ESP_LOGI(TAG, "Gyroscope full scale set to %d°/s",
             250 << gyro_fs);

    return ESP_OK;
}

esp_err_t mpu6500_set_accel_fs(mpu6500_t *mpu, mpu6500_accel_fs_t accel_fs)
{
    esp_err_t ret;
    uint8_t config;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read current config
    ret = mpu6500_read_reg(mpu, MPU6500_ACCEL_CONFIG, &config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Clear AFS_SEL bits and set new value
    config = (config & ~0x18) | (accel_fs << 3);

    ret = mpu6500_write_reg(mpu, MPU6500_ACCEL_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }

    mpu->accel_fs = accel_fs;
    mpu6500_calculate_scales(mpu);

    ESP_LOGI(TAG, "Accelerometer full scale set to ±%dg",
             2 << accel_fs);

    return ESP_OK;
}

esp_err_t mpu6500_set_dlpf(mpu6500_t *mpu, mpu6500_dlpf_t dlpf)
{
    esp_err_t ret;
    uint8_t config;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read current config
    ret = mpu6500_read_reg(mpu, MPU6500_CONFIG, &config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Clear DLPF_CFG bits and set new value
    config = (config & ~0x07) | dlpf;

    ret = mpu6500_write_reg(mpu, MPU6500_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "DLPF configured");
    return ESP_OK;
}

esp_err_t mpu6500_read_gyro_raw(mpu6500_t *mpu, mpu6500_raw_data_t *data)
{
    esp_err_t ret;
    uint8_t buf[6];

    if (mpu == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_regs(mpu, MPU6500_GYRO_XOUT_H, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    data->x = (int16_t)((buf[0] << 8) | buf[1]);
    data->y = (int16_t)((buf[2] << 8) | buf[3]);
    data->z = (int16_t)((buf[4] << 8) | buf[5]);

    return ESP_OK;
}

esp_err_t mpu6500_read_gyro(mpu6500_t *mpu, mpu6500_scaled_data_t *data)
{
    esp_err_t ret;
    mpu6500_raw_data_t raw;

    if (mpu == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_gyro_raw(mpu, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert to degrees per second
    data->x = (float)raw.x / mpu->gyro_scale;
    data->y = (float)raw.y / mpu->gyro_scale;
    data->z = (float)raw.z / mpu->gyro_scale;

    return ESP_OK;
}

esp_err_t mpu6500_read_accel_raw(mpu6500_t *mpu, mpu6500_raw_data_t *data)
{
    esp_err_t ret;
    uint8_t buf[6];

    if (mpu == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_regs(mpu, MPU6500_ACCEL_XOUT_H, buf, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    data->x = (int16_t)((buf[0] << 8) | buf[1]);
    data->y = (int16_t)((buf[2] << 8) | buf[3]);
    data->z = (int16_t)((buf[4] << 8) | buf[5]);

    return ESP_OK;
}

esp_err_t mpu6500_read_accel(mpu6500_t *mpu, mpu6500_scaled_data_t *data)
{
    esp_err_t ret;
    mpu6500_raw_data_t raw;

    if (mpu == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_accel_raw(mpu, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert to g
    data->x = (float)raw.x / mpu->accel_scale;
    data->y = (float)raw.y / mpu->accel_scale;
    data->z = (float)raw.z / mpu->accel_scale;

    return ESP_OK;
}

esp_err_t mpu6500_read_temperature(mpu6500_t *mpu, float *temp)
{
    esp_err_t ret;
    uint8_t buf[2];
    int16_t raw_temp;

    if (mpu == NULL || temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = mpu6500_read_regs(mpu, MPU6500_TEMP_OUT_H, buf, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    raw_temp = (int16_t)((buf[0] << 8) | buf[1]);

    // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/333.87 + 21
    *temp = (float)raw_temp / 333.87f + 21.0f;

    return ESP_OK;
}

esp_err_t mpu6500_read_all(mpu6500_t *mpu, mpu6500_scaled_data_t *accel,
                           mpu6500_scaled_data_t *gyro, float *temp)
{
    esp_err_t ret;
    uint8_t buf[14];
    mpu6500_raw_data_t accel_raw, gyro_raw;
    int16_t temp_raw;

    if (mpu == NULL || accel == NULL || gyro == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read all sensor data in one burst (accel, temp, gyro)
    ret = mpu6500_read_regs(mpu, MPU6500_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse accelerometer data
    accel_raw.x = (int16_t)((buf[0] << 8) | buf[1]);
    accel_raw.y = (int16_t)((buf[2] << 8) | buf[3]);
    accel_raw.z = (int16_t)((buf[4] << 8) | buf[5]);

    // Parse temperature data (only if requested)
    temp_raw = (int16_t)((buf[6] << 8) | buf[7]);

    // Parse gyroscope data
    gyro_raw.x = (int16_t)((buf[8] << 8) | buf[9]);
    gyro_raw.y = (int16_t)((buf[10] << 8) | buf[11]);
    gyro_raw.z = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to scaled values
    accel->x = (float)accel_raw.x / mpu->accel_scale;
    accel->y = (float)accel_raw.y / mpu->accel_scale;
    accel->z = (float)accel_raw.z / mpu->accel_scale;

    gyro->x = (float)gyro_raw.x / mpu->gyro_scale;
    gyro->y = (float)gyro_raw.y / mpu->gyro_scale;
    gyro->z = (float)gyro_raw.z / mpu->gyro_scale;

    // Only convert temperature if pointer provided
    if (temp != NULL) {
        *temp = (float)temp_raw / 333.87f + 21.0f;
    }

    return ESP_OK;
}

esp_err_t mpu6500_reset(mpu6500_t *mpu)
{
    esp_err_t ret;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set device reset bit
    ret = mpu6500_write_reg(mpu, MPU6500_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "MPU-6500 reset");
    return ESP_OK;
}

esp_err_t mpu6500_sleep(mpu6500_t *mpu, bool enable)
{
    esp_err_t ret;
    uint8_t config;

    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read current power management config
    ret = mpu6500_read_reg(mpu, MPU6500_PWR_MGMT_1, &config);
    if (ret != ESP_OK) {
        return ret;
    }

    if (enable) {
        config |= 0x40; // Set SLEEP bit
    } else {
        config &= ~0x40; // Clear SLEEP bit
    }

    ret = mpu6500_write_reg(mpu, MPU6500_PWR_MGMT_1, config);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "MPU-6500 %s", enable ? "sleep enabled" : "sleep disabled");
    return ESP_OK;
}

void mpu6500_get_accel_angles(mpu6500_scaled_data_t *accel, float *pitch, float *roll)
{
    if (accel == NULL || pitch == NULL || roll == NULL) {
        return;
    }

    // Calculate pitch (rotation around Y-axis)
    // atan2(x, sqrt(y^2 + z^2))
    *pitch = atan2(accel->x, sqrt(accel->y * accel->y + accel->z * accel->z)) * 180.0f / M_PI;

    // Calculate roll (rotation around X-axis)
    // atan2(y, sqrt(x^2 + z^2))
    *roll = atan2(accel->y, sqrt(accel->x * accel->x + accel->z * accel->z)) * 180.0f / M_PI;
}

void mpu6500_integrate_gyro_yaw(mpu6500_scaled_data_t *gyro, float dt, float *yaw)
{
    if (gyro == NULL || yaw == NULL) {
        return;
    }

    // Integrate gyroscope Z-axis (yaw rate) over time
    *yaw += gyro->z * dt;

    // Keep angle in range -180 to 180 degrees
    while (*yaw > 180.0f) {
        *yaw -= 360.0f;
    }
    while (*yaw < -180.0f) {
        *yaw += 360.0f;
    }
}

void mpu6500_get_pedal_position(mpu6500_scaled_data_t *accel,
                                float min_angle, float max_angle,
                                float *angle, float *percentage)
{
    if (accel == NULL || angle == NULL || percentage == NULL) {
        return;
    }

    // WARNING: This method is affected by vehicle acceleration!
    // Calculate the angle based on accelerometer reading
    // This assumes the sensor is mounted so that rotation is around one axis
    // You'll need to determine which calculation works for your mounting orientation

    // Option 1: If pedal rotates around X-axis (left-right pivot)
    // *angle = atan2(accel->y, accel->z) * 180.0f / M_PI;

    // Option 2: If pedal rotates around Y-axis (front-back pivot) - MOST COMMON
    *angle = atan2(accel->x, accel->z) * 180.0f / M_PI;

    // Option 3: If pedal rotates around Z-axis (twist)
    // *angle = atan2(accel->y, accel->x) * 180.0f / M_PI;

    // Convert angle to percentage (0-100%)
    // Clamp to min/max range
    float clamped_angle = *angle;
    if (clamped_angle < min_angle) clamped_angle = min_angle;
    if (clamped_angle > max_angle) clamped_angle = max_angle;

    // Map angle to percentage
    *percentage = ((clamped_angle - min_angle) / (max_angle - min_angle)) * 100.0f;

    // Ensure percentage is within 0-100%
    if (*percentage < 0.0f) *percentage = 0.0f;
    if (*percentage > 100.0f) *percentage = 100.0f;
}

void mpu6500_update_pedal_gyro(mpu6500_scaled_data_t *gyro, float dt,
                               float *current_angle,
                               float min_angle, float max_angle,
                               float *percentage)
{
    if (gyro == NULL || current_angle == NULL || percentage == NULL) {
        return;
    }

    // Integrate gyroscope to update angle
    // Using Y-axis gyro for typical pedal rotation (most common orientation)
    *current_angle += gyro->y * dt;

    // Handle both normal and reversed calibration
    bool reversed = (min_angle > max_angle);
    float actual_min = reversed ? max_angle : min_angle;
    float actual_max = reversed ? min_angle : max_angle;

    // Clamp to physical limits
    if (*current_angle < actual_min) *current_angle = actual_min;
    if (*current_angle > actual_max) *current_angle = actual_max;

    // Calculate percentage
    if (reversed) {
        // If reversed: min (released) is larger, max (pressed) is smaller
        // As angle decreases from min to max, percentage increases from 0 to 100
        *percentage = ((min_angle - *current_angle) / (min_angle - max_angle)) * 100.0f;
    } else {
        // Normal: as angle increases from min to max, percentage increases
        *percentage = ((*current_angle - min_angle) / (max_angle - min_angle)) * 100.0f;
    }

    // Clamp percentage to 0-100
    if (*percentage < 0.0f) *percentage = 0.0f;
    if (*percentage > 100.0f) *percentage = 100.0f;
}

void mpu6500_update_pedal_gyro_OLD_UNUSED(mpu6500_scaled_data_t *gyro, float dt,
                               float *current_angle,
                               float min_angle, float max_angle,
                               float *percentage)
{
    if (gyro == NULL || current_angle == NULL || percentage == NULL) {
        return;
    }

    // OLD CODE - NOT USED
    // Option 1: Pedal rotates around X-axis
    // *current_angle += gyro->x * dt;

    // Option 2: Pedal rotates around Y-axis (MOST COMMON)
    *current_angle += gyro->y * dt;

    // Option 3: Pedal rotates around Z-axis
    // *current_angle += gyro->z * dt;

    // Clamp angle to valid range (prevents drift beyond physical limits)
    if (*current_angle < min_angle) *current_angle = min_angle;
    if (*current_angle > max_angle) *current_angle = max_angle;

    // Convert angle to percentage
    *percentage = ((*current_angle - min_angle) / (max_angle - min_angle)) * 100.0f;

    // Ensure percentage is within 0-100%
    if (*percentage < 0.0f) *percentage = 0.0f;
    if (*percentage > 100.0f) *percentage = 100.0f;
}

void mpu6500_reset_pedal_angle(float *current_angle, float min_angle)
{
    if (current_angle == NULL) {
        return;
    }

    // Reset to rest position (pedal fully released)
    *current_angle = min_angle;
}

void mpu6500_show_orientation(mpu6500_t *mpu, uint32_t duration_ms, uint32_t update_rate_ms)
{
    if (mpu == NULL) {
        return;
    }

    if (update_rate_ms == 0) {
        update_rate_ms = 100; // Default 10Hz
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   MPU-6500 ORIENTATION DISPLAY");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Tilt the sensor to see orientation change");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    ESP_LOGI(TAG, "");

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;

    while (duration_ms == 0 || elapsed < duration_ms) {
        mpu6500_scaled_data_t accel;
        esp_err_t ret = mpu6500_read_accel(mpu, &accel);

        if (ret == ESP_OK) {
            // Calculate pitch and roll angles
            float pitch = atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0f / M_PI;
            float roll = atan2(accel.y, sqrt(accel.x * accel.x + accel.z * accel.z)) * 180.0f / M_PI;

            // Determine which face is up (strongest axis)
            float abs_x = fabsf(accel.x);
            float abs_y = fabsf(accel.y);
            float abs_z = fabsf(accel.z);

            const char *face_up = "UNKNOWN";
            if (abs_z > abs_x && abs_z > abs_y) {
                face_up = (accel.z > 0) ? "TOP (Z+)" : "BOTTOM (Z-)";
            } else if (abs_y > abs_x && abs_y > abs_z) {
                face_up = (accel.y > 0) ? "RIGHT (Y+)" : "LEFT (Y-)";
            } else {
                face_up = (accel.x > 0) ? "FRONT (X+)" : "BACK (X-)";
            }

            // Create tilt indicator (visual bar)
            char pitch_bar[21] = {0};
            char roll_bar[21] = {0};
            int pitch_pos = (int)((pitch + 90.0f) / 180.0f * 20.0f);
            int roll_pos = (int)((roll + 90.0f) / 180.0f * 20.0f);

            if (pitch_pos < 0) pitch_pos = 0;
            if (pitch_pos > 20) pitch_pos = 20;
            if (roll_pos < 0) roll_pos = 0;
            if (roll_pos > 20) roll_pos = 20;

            for (int i = 0; i < 21; i++) {
                pitch_bar[i] = (i == pitch_pos) ? '█' : (i == 10) ? '|' : '-';
                roll_bar[i] = (i == roll_pos) ? '█' : (i == 10) ? '|' : '-';
            }

            // Clear screen (ANSI escape code)
            printf("\033[2J\033[H");

            // Display orientation
            printf("╔════════════════════════════════════════╗\n");
            printf("║   MPU-6500 ORIENTATION DISPLAY         ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║                                        ║\n");
            printf("║  Face Up: %-28s ║\n", face_up);
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  PITCH: %6.1f°                        ║\n", pitch);
            printf("║  [%s]  ║\n", pitch_bar);
            printf("║   -90°         0°          +90°        ║\n");
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  ROLL:  %6.1f°                        ║\n", roll);
            printf("║  [%s]  ║\n", roll_bar);
            printf("║   -90°         0°          +90°        ║\n");
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  Accel (g):                            ║\n");
            printf("║    X: %6.2f  Y: %6.2f  Z: %6.2f   ║\n", accel.x, accel.y, accel.z);
            printf("║                                        ║\n");

            // Draw a simple 3D box representation
            printf("╠════════════════════════════════════════╣\n");
            printf("║          3D Orientation:               ║\n");
            printf("║                                        ║\n");

            // Simplified 3D box based on orientation
            if (fabsf(accel.z) > 0.7f) {
                // Flat (Z-axis dominant)
                if (accel.z > 0) {
                    printf("║            ┌─────────┐                ║\n");
                    printf("║            │  TOP    │                ║\n");
                    printf("║            │    ▲    │                ║\n");
                    printf("║            │    Z    │                ║\n");
                    printf("║            └─────────┘                ║\n");
                } else {
                    printf("║            ┌─────────┐                ║\n");
                    printf("║            │    Z    │                ║\n");
                    printf("║            │    ▼    │                ║\n");
                    printf("║            │ BOTTOM  │                ║\n");
                    printf("║            └─────────┘                ║\n");
                }
            } else if (fabsf(accel.y) > 0.7f) {
                // Tilted on Y-axis
                if (accel.y > 0) {
                    printf("║               ╱│                      ║\n");
                    printf("║              ╱ │ Y+                   ║\n");
                    printf("║             ╱  │                      ║\n");
                    printf("║            ╱   │                      ║\n");
                    printf("║           ─────┘                      ║\n");
                } else {
                    printf("║           └─────                      ║\n");
                    printf("║            │   ╲                      ║\n");
                    printf("║            │  Y-╲                     ║\n");
                    printf("║            │     ╲                    ║\n");
                    printf("║            │      ╲                   ║\n");
                }
            } else {
                // Tilted on X-axis
                if (accel.x > 0) {
                    printf("║              ┌───┐                    ║\n");
                    printf("║              │ X+│                    ║\n");
                    printf("║              │ ▶ │                    ║\n");
                    printf("║              │   │                    ║\n");
                    printf("║              └───┘                    ║\n");
                } else {
                    printf("║              ┌───┐                    ║\n");
                    printf("║              │   │                    ║\n");
                    printf("║              │ ◀ │                    ║\n");
                    printf("║              │X- │                    ║\n");
                    printf("║              └───┘                    ║\n");
                }
            }

            printf("║                                        ║\n");
            printf("╚════════════════════════════════════════╝\n");

            fflush(stdout);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer");
        }

        vTaskDelay(pdMS_TO_TICKS(update_rate_ms));
        elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
    }

    ESP_LOGI(TAG, "Orientation display stopped");
}

void mpu6500_show_orientation_gyro(mpu6500_t *mpu, uint32_t duration_ms, uint32_t update_rate_ms, float alpha)
{
    if (mpu == NULL) {
        return;
    }

    if (update_rate_ms == 0) {
        update_rate_ms = 50; // Default 20Hz
    }

    if (alpha == 0.0f || alpha < 0.9f || alpha > 0.999f) {
        alpha = 0.98f; // Default complementary filter coefficient
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   MPU-6500 GYRO ORIENTATION DISPLAY");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Using complementary filter (alpha=%.3f)", alpha);
    ESP_LOGI(TAG, "Works in moving vehicle!");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    ESP_LOGI(TAG, "");

    // Initialize angles from accelerometer
    mpu6500_scaled_data_t accel, gyro;
    esp_err_t ret = mpu6500_read_accel(mpu, &accel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize orientation");
        return;
    }

    float pitch = atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0f / M_PI;
    float roll = atan2(accel.y, sqrt(accel.x * accel.x + accel.z * accel.z)) * 180.0f / M_PI;

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = 0;
    float dt = update_rate_ms / 1000.0f; // Convert to seconds

    while (duration_ms == 0 || elapsed < duration_ms) {
        ret = mpu6500_read_all(mpu, &accel, &gyro, NULL);

        if (ret == ESP_OK) {
            // Calculate accelerometer angles
            float accel_pitch = atan2(accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0f / M_PI;
            float accel_roll = atan2(accel.y, sqrt(accel.x * accel.x + accel.z * accel.z)) * 180.0f / M_PI;

            // Integrate gyroscope
            float gyro_pitch_delta = gyro.x * dt;
            float gyro_roll_delta = gyro.y * dt;

            // Complementary filter: combine gyro and accel
            pitch = alpha * (pitch + gyro_pitch_delta) + (1.0f - alpha) * accel_pitch;
            roll = alpha * (roll + gyro_roll_delta) + (1.0f - alpha) * accel_roll;

            // Determine which face is up based on filtered angles
            const char *face_up = "UNKNOWN";
            if (fabsf(pitch) < 30.0f && fabsf(roll) < 30.0f) {
                face_up = "TOP (Z+)";
            } else if (fabsf(pitch) > 150.0f || fabsf(roll) > 150.0f) {
                face_up = "BOTTOM (Z-)";
            } else if (roll > 60.0f) {
                face_up = "RIGHT (Y+)";
            } else if (roll < -60.0f) {
                face_up = "LEFT (Y-)";
            } else if (pitch > 60.0f) {
                face_up = "FRONT (X+)";
            } else if (pitch < -60.0f) {
                face_up = "BACK (X-)";
            }

            // Create tilt indicator bars
            char pitch_bar[21] = {0};
            char roll_bar[21] = {0};
            int pitch_pos = (int)((pitch + 90.0f) / 180.0f * 20.0f);
            int roll_pos = (int)((roll + 90.0f) / 180.0f * 20.0f);

            if (pitch_pos < 0) pitch_pos = 0;
            if (pitch_pos > 20) pitch_pos = 20;
            if (roll_pos < 0) roll_pos = 0;
            if (roll_pos > 20) roll_pos = 20;

            for (int i = 0; i < 21; i++) {
                pitch_bar[i] = (i == pitch_pos) ? '█' : (i == 10) ? '|' : '-';
                roll_bar[i] = (i == roll_pos) ? '█' : (i == 10) ? '|' : '-';
            }

            // Clear screen
            printf("\033[2J\033[H");

            // Display orientation
            printf("╔════════════════════════════════════════╗\n");
            printf("║   GYRO ORIENTATION (Vehicle Mode)     ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║                                        ║\n");
            printf("║  Face Up: %-28s ║\n", face_up);
            printf("║  Filter: %d%% Gyro, %d%% Accel           ║\n",
                   (int)(alpha * 100), (int)((1.0f - alpha) * 100));
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  PITCH: %6.1f°                        ║\n", pitch);
            printf("║  [%s]  ║\n", pitch_bar);
            printf("║   -90°         0°          +90°        ║\n");
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  ROLL:  %6.1f°                        ║\n", roll);
            printf("║  [%s]  ║\n", roll_bar);
            printf("║   -90°         0°          +90°        ║\n");
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  Gyro Rate (°/s):                      ║\n");
            printf("║    X: %6.1f  Y: %6.1f  Z: %6.1f    ║\n", gyro.x, gyro.y, gyro.z);
            printf("║                                        ║\n");
            printf("║  Accel (g):                            ║\n");
            printf("║    X: %6.2f  Y: %6.2f  Z: %6.2f   ║\n", accel.x, accel.y, accel.z);
            printf("║                                        ║\n");
            printf("╠════════════════════════════════════════╣\n");
            printf("║  Note: Angles fused from gyro + accel ║\n");
            printf("║  Safe for use while vehicle moving!   ║\n");
            printf("╚════════════════════════════════════════╝\n");

            fflush(stdout);
        } else {
            ESP_LOGE(TAG, "Failed to read sensors");
        }

        vTaskDelay(pdMS_TO_TICKS(update_rate_ms));
        elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
    }

    ESP_LOGI(TAG, "Gyro orientation display stopped");
}
