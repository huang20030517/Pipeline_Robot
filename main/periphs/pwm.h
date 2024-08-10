#include <stdint.h>

#include "driver/ledc.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 舵机 500 2500us

// 舵机最小角度
#define SERVO_MIN_ANGLE -20
// 舵机最大角度
#define SERVO_MAX_ANGLE 50

#define SERVO_GPIO 35    // 连接舵机的 GPIO 引脚
#define SERVO_FREQ_HZ 50 // 舵机控制频率 50Hz

// 直流电机

#define DC0_GPIO_PH 5 // 0-反向 1-正向
#define DC0_GPIO_EN 6

#define DC1_GPIO_PH 33 // 0-反向 1-正向
#define DC1_GPIO_EN 34

#define DC0_CHANNEL LEDC_CHANNEL_1
#define DC1_CHANNEL LEDC_CHANNEL_2

void pwm_init(void);

void set_servo_angle(uint8_t angle);
void set_motor_direction(float x, float y);

#ifdef __cplusplus
}
#endif
