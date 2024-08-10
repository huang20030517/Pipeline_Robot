#include "mpu6050.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

void i2c_sensor_mpu6050_init(void);

void mpu6050_deviceid(uint8_t *deviceid);

void mpu6050_acce(mpu6050_acce_value_t *acce);

void mpu6050_gyro(mpu6050_gyro_value_t *gyro);

void mpu6050_temp(mpu6050_temp_value_t *temp);

void mpu6050_angle(complimentary_angle_t *angle);


#ifdef __cplusplus
}
#endif

