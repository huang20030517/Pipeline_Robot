#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "mpu6050.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief I2C 配置参数
 */
#define I2C_MASTER_SCL_IO 34      ///< I2C 时钟 GPIO
#define I2C_MASTER_SDA_IO 33      ///< I2C 数据 GPIO
#define I2C_MASTER_NUM I2C_NUM_0  ///< I2C 端口号
#define I2C_MASTER_FREQ_HZ 100000 ///< I2C 时钟频率（100kHz）

/**
 * @brief 初始化 MPU6050 传感器
 * @note 包括 I2C 总线配置和 MPU6050 参数配置
 */
void i2c_sensor_mpu6050_init(void);

/**
 * @brief 获取 MPU6050 的设备 ID
 * @param[out] deviceid 存储设备 ID 的指针
 */
void mpu6050_deviceid(uint8_t *deviceid);

/**
 * @brief 获取加速度值
 * @param[out] acce 存储加速度值的结构体指针
 */
void mpu6050_acce(mpu6050_acce_value_t *acce);

/**
 * @brief 获取陀螺仪值
 * @param[out] gyro 存储陀螺仪值的结构体指针
 */
void mpu6050_gyro(mpu6050_gyro_value_t *gyro);

/**
 * @brief 获取温度值
 * @param[out] temp 存储温度值的结构体指针
 */
void mpu6050_temp(mpu6050_temp_value_t *temp);

/**
 * @brief 计算姿态角（融合加速度和陀螺仪数据）
 * @param[out] angle 存储计算得到的姿态角
 */
void mpu6050_angle(complimentary_angle_t *angle);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_DRIVER_H
