#include "mpu6050.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_MASTER_SCL_IO 34      // I2C 时钟 GPIO
#define I2C_MASTER_SDA_IO 33      // I2C 数据 GPIO
#define I2C_MASTER_NUM I2C_NUM_0  // I2C 端口号
#define I2C_MASTER_FREQ_HZ 100000 // I2C 时钟频率（100kHz）


    void i2c_sensor_mpu6050_init(void);
    void mpu6050_deviceid(uint8_t *deviceid);
    void mpu6050_acce(mpu6050_acce_value_t *acce);
    void mpu6050_gyro(mpu6050_gyro_value_t *gyro);
    void mpu6050_temp(mpu6050_temp_value_t *temp);
    void mpu6050_angle(complimentary_angle_t *angle);

#ifdef __cplusplus
}
#endif
