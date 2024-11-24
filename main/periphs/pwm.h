#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/mcpwm_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 舵机配置
 */
#define SERVO_GPIO 9                  ///< 舵机连接的 GPIO 引脚
#define SERVO_FREQ_HZ 50              ///< 舵机 PWM 频率
#define SERVO_TIMER LEDC_TIMER_0      ///< 使用的 LEDC Timer
#define SERVO_CHANNEL LEDC_CHANNEL_0  ///< 使用的 LEDC 通道
#define SERVO_MAX_PULSE_WIDTH_US 2500 ///< 最大脉宽 (us)
#define SERVO_MIN_PULSE_WIDTH_US 500  ///< 最小脉宽 (us)

/**
 * @brief 灯光配置
 */
#define LIGHT_GPIO 11                ///< 灯光连接的 GPIO 引脚
#define LIGHT_FREQ_HZ 2000           ///< 灯光 PWM 频率
#define LIGHT_TIMER LEDC_TIMER_2     ///< 使用的 LEDC Timer
#define LIGHT_CHANNEL LEDC_CHANNEL_3 ///< 使用的 LEDC 通道

/**
 * @brief 直流电机 0 配置
 */
#define DC0_GPIO_PH 1              ///< DC0 方向控制引脚
#define DC0_GPIO_EN 2              ///< DC0 使能引脚
#define DC0_TIMER LEDC_TIMER_1     ///< DC0 使用的 LEDC Timer
#define DC0_FREQ_HZ 30000          ///< DC0 PWM 频率
#define DC0_CHANNEL LEDC_CHANNEL_1 ///< DC0 使用的 LEDC 通道

/**
 * @brief 直流电机 1 配置
 */
#define DC1_GPIO_PH 7              ///< DC1 方向控制引脚
#define DC1_GPIO_EN 8              ///< DC1 使能引脚
#define DC1_TIMER LEDC_TIMER_1     ///< DC1 使用的 LEDC Timer
#define DC1_FREQ_HZ 30000          ///< DC1 PWM 频率
#define DC1_CHANNEL LEDC_CHANNEL_2 ///< DC1 使用的 LEDC 通道

/**
 * @brief 初始化 PWM 模块
 */
void pwm_init(void);

/**
 * @brief 设置舵机角度
 * @param angle 舵机角度，范围为 0 到 180
 */
void set_servo_angle(uint8_t angle);

/**
 * @brief 设置电机速度和方向
 * @param x X 轴速度，范围为 -1.0 到 1.0
 * @param y Y 轴速度，范围为 -1.0 到 1.0
 */
void set_motor_direction(float x, float y);

/**
 * @brief 设置灯光亮度
 * @param brightness 灯光亮度，范围为 0 到 100
 */
void set_light_brightness(uint8_t brightness);

#ifdef __cplusplus
}
#endif

#endif // PWM_DRIVER_H
