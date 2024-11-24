#include "pwm.h"

/// @brief 初始化 LEDC Timer 和 Channel
/// @param timer LEDC Timer 配置
/// @param channel LEDC Channel 配置
static void configure_ledc(ledc_timer_config_t *timer, ledc_channel_config_t *channel)
{
    ESP_ERROR_CHECK(ledc_timer_config(timer));
    ESP_ERROR_CHECK(ledc_channel_config(channel));
}

/// @brief 初始化 PWM 控制，包括舵机、灯光和直流电机
void pwm_init(void)
{
    // 舵机配置
    configure_ledc(
        &(ledc_timer_config_t){
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_14_BIT,
            .timer_num = SERVO_TIMER,
            .freq_hz = SERVO_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK},
        &(ledc_channel_config_t){
            .gpio_num = SERVO_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SERVO_CHANNEL,
            .timer_sel = SERVO_TIMER,
            .duty = 0,
            .hpoint = 0});

    // 灯光配置
    configure_ledc(
        &(ledc_timer_config_t){
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_14_BIT,
            .timer_num = LIGHT_TIMER,
            .freq_hz = LIGHT_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK},
        &(ledc_channel_config_t){
            .gpio_num = LIGHT_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LIGHT_CHANNEL,
            .timer_sel = LIGHT_TIMER,
            .duty = 0,
            .hpoint = 0});

    // 直流电机 GPIO 配置
    ESP_ERROR_CHECK(gpio_config(&(gpio_config_t){
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << DC0_GPIO_PH) | (1ULL << DC1_GPIO_PH)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE}));

    // DC0 配置
    configure_ledc(
        &(ledc_timer_config_t){
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_11_BIT,
            .timer_num = DC0_TIMER,
            .freq_hz = DC0_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK},
        &(ledc_channel_config_t){
            .gpio_num = DC0_GPIO_EN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = DC0_CHANNEL,
            .timer_sel = DC0_TIMER,
            .duty = 0,
            .hpoint = 0});

    // DC1 配置
    configure_ledc(
        &(ledc_timer_config_t){
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_11_BIT,
            .timer_num = DC1_TIMER,
            .freq_hz = DC1_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK},
        &(ledc_channel_config_t){
            .gpio_num = DC1_GPIO_EN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = DC1_CHANNEL,
            .timer_sel = DC1_TIMER,
            .duty = 0,
            .hpoint = 0});

    // 停止直流电机初始输出
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, DC0_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, DC1_CHANNEL, 0));
}

/// @brief 限制值在指定的范围内
static inline uint8_t clamp(uint8_t value, uint8_t min, uint8_t max)
{
    return (value < min) ? min : (value > max) ? max
                                               : value;
}

/// @brief 设置直流电机的速度和方向
void set_motor_speed_and_direction(int16_t left_speed, int16_t right_speed)
{
    // 右轮控制
    gpio_set_level(DC0_GPIO_PH, right_speed >= 0);
    right_speed = abs(right_speed);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, DC0_CHANNEL, right_speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, DC0_CHANNEL));

    // 左轮控制
    gpio_set_level(DC1_GPIO_PH, left_speed >= 0);
    left_speed = abs(left_speed);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, DC1_CHANNEL, left_speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, DC1_CHANNEL));
}

/// @brief 设置直流电机方向
void set_motor_direction(float x, float y)
{
    const float min_effective_speed = 0.4f;
    float left_wheel_velocity = y + x;
    float right_wheel_velocity = y - x;

    left_wheel_velocity = fmax(-2047.0f, fmin(2047.0f, left_wheel_velocity * 2047.0f));
    right_wheel_velocity = fmax(-2047.0f, fmin(2047.0f, right_wheel_velocity * 2047.0f));

    if (fabs(left_wheel_velocity) > 0 && fabs(left_wheel_velocity) < min_effective_speed)
    {
        left_wheel_velocity = (left_wheel_velocity > 0) ? min_effective_speed : -min_effective_speed;
    }
    if (fabs(right_wheel_velocity) > 0 && fabs(right_wheel_velocity) < min_effective_speed)
    {
        right_wheel_velocity = (right_wheel_velocity > 0) ? min_effective_speed : -min_effective_speed;
    }

    set_motor_speed_and_direction(left_wheel_velocity, right_wheel_velocity);
}

/// @brief 设置舵机角度
void set_servo_angle(uint8_t angle)
{
    angle = clamp(angle, 45, 115);

    uint32_t pulse_width_us = SERVO_MIN_PULSE_WIDTH_US +
                              (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US) * angle / 180;

    uint32_t duty = (pulse_width_us * (1 << LEDC_TIMER_14_BIT)) / (1000000 / SERVO_FREQ_HZ);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL));
}

/// @brief 设置灯光亮度
void set_light_brightness(uint8_t brightness)
{
    brightness = clamp(brightness, 0, 100);
    uint32_t duty = (brightness * ((1 << LEDC_TIMER_14_BIT) - 1)) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LIGHT_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LIGHT_CHANNEL));
}
