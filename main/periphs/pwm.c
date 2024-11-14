#include "pwm.h"

void pwm_init(void)
{
    // 舵机配置
    ESP_ERROR_CHECK(ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK}));

    ESP_ERROR_CHECK(ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0}));

    // 灯光设置
    ESP_ERROR_CHECK(ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LIGHT_TIMER,
        .freq_hz = LIGHT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK}));

    ESP_ERROR_CHECK(ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num = LIGHT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LIGHT_CHANNEL,
        .timer_sel = LIGHT_TIMER,
        .duty = 0,
        .hpoint = 0}));

    // 直流配置
    gpio_config_t DC_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << DC0_GPIO_PH) | (1ULL << DC1_GPIO_PH)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};

    ESP_ERROR_CHECK(gpio_config(&DC_conf));

    // DC0
    ledc_timer_config_t DC0_tiner = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .timer_num = DC0_TIMER,
        .freq_hz = DC0_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&DC0_tiner));

    ledc_channel_config_t DC0_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = DC0_CHANNEL,
        .timer_sel = DC0_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = DC0_GPIO_EN,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&DC0_config));

    // DC1
    ledc_timer_config_t DC1_tiner = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .timer_num = DC1_TIMER,
        .freq_hz = DC1_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&DC1_tiner));

    ledc_channel_config_t DC1_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = DC1_CHANNEL,
        .timer_sel = DC1_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = DC1_GPIO_EN,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&DC1_config));

    ESP_ERROR_CHECK((ledc_stop(LEDC_LOW_SPEED_MODE, DC0_CHANNEL, 0)));
    ESP_ERROR_CHECK((ledc_stop(LEDC_LOW_SPEED_MODE, DC1_CHANNEL, 0)));
}

//---------------------------------------------------------------------------------------//

/// @brief 限制值在指定的范围内
/// @param value 要限制的值
/// @param min 最小值
/// @param max 最大值
/// @return 限制后的值
static inline uint8_t clamp(uint8_t value, uint8_t min, uint8_t max)
{
    return (value < min) ? min : (value > max) ? max
                                               : value;
}
void set_motor_speed_and_direction(int16_t left_speed, int16_t right_speed)
{

    // 右轮控制
    if (right_speed >= 0)
    {
        gpio_set_level(DC0_GPIO_PH, 1); // 设置右轮前进
    }
    else
    {
        gpio_set_level(DC0_GPIO_PH, 0); // 设置右轮后退
        right_speed = -right_speed;     // 取绝对值
    }

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, DC0_CHANNEL, right_speed)); // 设置 PWM 占空比
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, DC0_CHANNEL));

    // 左轮控制
    if (left_speed >= 0)
    {
        gpio_set_level(DC1_GPIO_PH, 1); // 设置左轮前进
    }
    else
    {
        gpio_set_level(DC1_GPIO_PH, 0); // 设置左轮后退
        left_speed = -left_speed;       // 取绝对值
    }

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, DC1_CHANNEL, left_speed)); // 设置 PWM 占空比
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, DC1_CHANNEL));
}

void set_motor_direction(float x, float y)
{
    const float min_effective_speed = 0.4f;
    float left_wheel_velocity = y + x;
    float right_wheel_velocity = y - x;

    // 判断速度是否在有效范围内，如果在则设为最小有效速度
    if (fabs(left_wheel_velocity) > 0 && fabs(left_wheel_velocity) < min_effective_speed)
    {
        left_wheel_velocity = (left_wheel_velocity > 0) ? min_effective_speed : -min_effective_speed;
    }
    if (fabs(right_wheel_velocity) > 0 && fabs(right_wheel_velocity) < min_effective_speed)
    {
        right_wheel_velocity = (right_wheel_velocity > 0) ? min_effective_speed : -min_effective_speed;
    }

    left_wheel_velocity = fmax(-2047.0f, fmin(2047.f, left_wheel_velocity * 2047.0f));
    right_wheel_velocity = fmax(-2047.0f, fmin(2047.f, right_wheel_velocity * 2047.0f));

    set_motor_speed_and_direction(left_wheel_velocity, right_wheel_velocity);
}

/// @brief  设置舵机角度的函数
/// @param angle 0 到 100 范围内
void set_servo_angle(uint8_t angle)
{
    // 45 90 115
    if (angle >= 115)
        angle = 115;
    else if (angle <= 45)
        angle = 45;

    angle = clamp(angle, 0, 180);

    // 计算对应的脉宽
    uint32_t pulse_width_us = SERVO_MIN_PULSE_WIDTH_US + (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US) * angle / 180;

    // 计算 PWM 占空比
    uint32_t duty = (pulse_width_us * (1 << LEDC_TIMER_14_BIT)) / (1000000 / SERVO_FREQ_HZ);

    // 如果 PWM 输出反向，调整占空比
    uint32_t max_duty = (1 << LEDC_TIMER_14_BIT) - 1;
    uint32_t reversed_duty = max_duty - duty;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, reversed_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

/// @brief 设置灯光亮度的函数
/// @param brightness  0 到 100 范围内
void set_light_brightness(uint8_t brightness)
{

    brightness = brightness > 100 ? 100 : brightness;

    uint32_t duty = (brightness * (1 << LEDC_TIMER_14_BIT) - 1) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LIGHT_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LIGHT_CHANNEL));
}