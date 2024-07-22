#include "periphs/mpu6050_driver.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"




#define I2C_MASTER_SCL_IO 14      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 13      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}


// 读取数据

void mpu6050_deviceid(uint8_t *deviceid)
{
    esp_err_t ret = mpu6050_get_deviceid(mpu6050, deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void mpu6050_acce(mpu6050_acce_value_t *acce) 
{
    esp_err_t ret = mpu6050_get_acce(mpu6050, acce);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void mpu6050_gyro(mpu6050_gyro_value_t *gyro)
{
    esp_err_t ret = mpu6050_get_gyro(mpu6050, gyro);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void mpu6050_temp(mpu6050_temp_value_t *temp)
{
    esp_err_t ret = mpu6050_get_temp(mpu6050, temp);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void mpu6050_angle(complimentary_angle_t *angle)
{  
    mpu6050_gyro_value_t gyro;
    mpu6050_acce_value_t acce;

    mpu6050_acce(&acce);
    mpu6050_gyro(&gyro);

    esp_err_t ret = mpu6050_complimentory_filter(mpu6050, &acce, &gyro, angle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

}
