#include "periphs/mpu6050_driver.h"
#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

mpu6050_handle_t mpu6050 = NULL; ///< MPU6050 句柄

/**
 * @brief 初始化 I2C 总线
 * @note 此函数完成 I2C 配置并安装驱动
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 初始化 MPU6050 传感器
 * @note 包括创建 MPU6050 句柄和配置参数
 */
void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    // 初始化 I2C 总线
    i2c_bus_init();

    // 创建 MPU6050 句柄
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (!mpu6050)
    {
        ESP_LOGE(TAG, "Failed to create MPU6050 handle");
        return;
    }

    // 配置加速度和陀螺仪量程
    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 config failed: %s", esp_err_to_name(ret));
        return;
    }

    // 唤醒 MPU6050
    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 wake-up failed: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 获取 MPU6050 的设备 ID
 * @param[out] deviceid 存储设备 ID 的指针
 */
void mpu6050_deviceid(uint8_t *deviceid)
{
    esp_err_t ret = mpu6050_get_deviceid(mpu6050, deviceid);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get device ID: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 获取加速度值
 * @param[out] acce 存储加速度值的结构体指针
 */
void mpu6050_acce(mpu6050_acce_value_t *acce)
{
    esp_err_t ret = mpu6050_get_acce(mpu6050, acce);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get acceleration: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 获取陀螺仪值
 * @param[out] gyro 存储陀螺仪值的结构体指针
 */
void mpu6050_gyro(mpu6050_gyro_value_t *gyro)
{
    esp_err_t ret = mpu6050_get_gyro(mpu6050, gyro);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get gyroscope: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 获取温度值
 * @param[out] temp 存储温度值的结构体指针
 */
void mpu6050_temp(mpu6050_temp_value_t *temp)
{
    esp_err_t ret = mpu6050_get_temp(mpu6050, temp);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get temperature: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 计算姿态角（融合加速度和陀螺仪数据）
 * @param[out] angle 存储计算得到的姿态角
 */
void mpu6050_angle(complimentary_angle_t *angle)
{
    mpu6050_gyro_value_t gyro;
    mpu6050_acce_value_t acce;

    // 获取加速度和陀螺仪数据
    mpu6050_acce(&acce);
    mpu6050_gyro(&gyro);

    // 计算融合角度
    esp_err_t ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, angle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to compute complimentary filter: %s", esp_err_to_name(ret));
    }
}
