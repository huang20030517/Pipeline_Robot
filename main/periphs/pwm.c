#include "pwm.h"
#include "math.h"
#include "esp_log.h"
#include "stdio.h"

#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "driver/gpio.h"


void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    // ledc_timer_config_t ledc_timer = {
    //     .speed_mode       = LEDC_MODE,
    //     .timer_num        = LEDC_TIMER,
    //     .duty_resolution  = LEDC_DUTY_RES,
    //     .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
    //     .clk_cfg          = LEDC_AUTO_CLK
    // };
    // ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // // Prepare and then apply the LEDC PWM channel configuration
    // ledc_channel_config_t ledc_channel = {
    //     .speed_mode     = LEDC_MODE,
    //     .channel        = LEDC_CHANNEL_0,
    //     .timer_sel      = LEDC_TIMER,
    //     .intr_type      = LEDC_INTR_DISABLE,
    //     .gpio_num       = LEDC_OUTPUT_IO,
    //     .duty           = 0, // Set duty to 0%
    //     .hpoint         = 0
    // };
    // ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

    // 步进电机1 
    ledc_channel_config_t stepper1_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = M1_STEP_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&stepper1_channel);

    ledc_timer_config_t stepper1_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // 分辨率 8 位
        .freq_hz         = 2000,           
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0
    };
    ledc_timer_config(&stepper1_timer); 
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));


    // 步进电机3
    ledc_channel_config_t stepper2_channel = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = M3_STEP_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&stepper2_channel);

    ledc_timer_config_t stepper2_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // 分辨率 8 位
        .freq_hz         = 2000,           
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0
    };
    ledc_timer_config(&stepper2_timer); 
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));

    // 
    gpio_config_t io_conf = {
        .pin_bit_mask =  (1ull << M1_DIR_GPIO) | (1ull << M1_ENN_GPIO) | (1ull << M3_DIR_GPIO) | (1ull << M3_ENN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);


    // 直流电机的初始
    // MCPWM 定时器 0 配置
    mcpwm_config_t pwm_config_0 = {
        .frequency = 10000, 
        .cmpr_a = 0,          // A相占空比初始为 0%
        .cmpr_b = 0,          // B相占空比初始为 0%
        .duty_mode = MCPWM_DUTY_MODE_0, // 正常模式
        .counter_mode = MCPWM_UP_COUNTER  // 向上计数模式
    };
    
    // MCPWM 定时器 1 配置 
    mcpwm_config_t pwm_config_1 = {
        .frequency = 10000, 
        .cmpr_a = 0,          // A相占空比初始为 0%
        .cmpr_b = 0,          // B相占空比初始为 0%
        .duty_mode = MCPWM_DUTY_MODE_0, // 正常模式
        .counter_mode = MCPWM_UP_COUNTER  // 向上计数模式
    };

    // MCPWM 引脚配置
    mcpwm_pin_config_t mcpwm_pins = {
        .mcpwm0a_out_num = 14,
        .mcpwm0b_out_num = 13,
        .mcpwm1a_out_num = 42,
        .mcpwm1b_out_num = 41
    };

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_0));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_1));

    // 配置 MCPWM 引脚
    ESP_ERROR_CHECK(mcpwm_set_pin(MCPWM_UNIT_0, &mcpwm_pins));
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1)); 

}


void control_Pitch_motor(float angle)
{
    static const char *TAG = "电机控制";

    // 计算需要的步数
    int steps = (int)((angle / 360.0) * 16000);
    // 设置方向
    // ESP_LOGI(TAG, "angle : %0.2f    steps : %d", angle,  steps);
    if (angle >= 0) {
        gpio_set_level(M3_DIR_GPIO, 1); // 正方向
    } else {
        gpio_set_level(M3_DIR_GPIO, 0); // 反方向
    }
    // 生成脉冲信号,驱动步进电机转动到指定角度
    for (int i = 0; i < abs(steps); i++)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 128); // 50% 占空比
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); // 关闭脉冲
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void control_gimbal_motor(float angle)
{
    static const char *TAG = "电机控制";

    // 计算需要的步数
    int steps = (int)((angle / 360.0) * 16000);
    // 设置方向
    // ESP_LOGI(TAG, "angle : %0.2f    steps : %d", angle,  steps);
    if (angle >= 0) {
        gpio_set_level(M1_DIR_GPIO, 1); // 正方向
    } else {
        gpio_set_level(M1_DIR_GPIO, 0); // 反方向
    }
    // 生成脉冲信号,驱动步进电机转动到指定角度
    for (int i = 0; i < abs(steps); i++)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128); // 50% 占空比
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // 关闭脉冲
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}


void set_motor_direction(float x, float y)
{
    static const char *TAG = "电机控制";

    // 计算左右电机的目标速度
    float left_wheel_velocity = y + x;
    float right_wheel_velocity = y - x;

    left_wheel_velocity = fmax(-1000.0f, fmin(1000.f, left_wheel_velocity * 200.0f));
    right_wheel_velocity = fmax(-1000.0f, fmin(1000.f, right_wheel_velocity * 200.0f));
    
    // ESP_LOGI(TAG, "left : %0.3f, right : %0.3f", left_wheel_velocity, right_wheel_velocity);

    if (left_wheel_velocity >= 0)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, left_wheel_velocity);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    }
    else
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -left_wheel_velocity);
    }

    if (right_wheel_velocity >= 0)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, right_wheel_velocity);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    }
    else
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, -right_wheel_velocity);
    }
}
