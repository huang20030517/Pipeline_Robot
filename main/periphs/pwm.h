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

// 舵机 500 2500us
// 舵机配置
#define SERVO_GPIO 9                  // 舵机连接的 GPIO 引脚
#define SERVO_FREQ_HZ 50              // 舵机频率 50Hz
#define SERVO_MIN_ANGLE -20           // 最小角度
#define SERVO_MAX_ANGLE 50            // 最大角度
#define SERVO_TIMER LEDC_TIMER_0      // 使用 LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0  // 使用通道 0
#define SERVO_MAX_PULSE_WIDTH_US 2500 // 最大脉宽
#define SERVO_MIN_PULSE_WIDTH_US 500  // 最小脉宽

// 灯光配置
#define LIGHT_GPIO 11                // 灯光连接的 GPIO 引脚
#define LIGHT_TIMER LEDC_TIMER_2     // 灯光使用的 LEDC 计时器
#define LIGHT_CHANNEL LEDC_CHANNEL_3 // 灯光使用的 LEDC 通道
#define LIGHT_FREQ_HZ 2000           // 灯光的 PWM 频率

// 直流电机0配置
#define DC0_GPIO_PH 1              // DC0 方向控制引脚
#define DC0_GPIO_EN 2              // DC0 使能引脚
#define DC0_TIMER LEDC_TIMER_1     // DC0 使用的 LEDC 计时器
#define DC0_FREQ_HZ 30000          // DC0 频率 30kHz
#define DC0_CHANNEL LEDC_CHANNEL_1 // DC0 使用的通道 1

// 直流电机1配置
#define DC1_GPIO_PH 7              // DC1 方向控制引脚
#define DC1_GPIO_EN 8              // DC1 使能引脚
#define DC1_TIMER LEDC_TIMER_1     // DC1 使用的 LEDC 计时器
#define DC1_FREQ_HZ 30000          // DC1 频率 30kHz
#define DC1_CHANNEL LEDC_CHANNEL_2 // DC1 使用的通道 2

    void pwm_init(void);

    void set_servo_angle(uint8_t angle);
    void set_motor_direction(float x, float y);
    void set_light_brightness(uint8_t brightness);
#ifdef __cplusplus
}
#endif
