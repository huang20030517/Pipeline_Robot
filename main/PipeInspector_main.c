/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "periphs/mpu6050_driver.c"
#include "periphs/usb.h"
#include "periphs/pwm.h"
#include "periphs/can.h"
#include "periphs/encoder.h"
#include "periphs/led.h"


QueueHandle_t encoder_event_queue;
static NormalizedRemoteControlData_t RC_Normalized_data; 
RC_t rc_data;

#define BLINK_GPIO 48

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    if(rc_data.led_brightness > 0 )
    {
        led_strip_set_pixel(led_strip, 0, rc_data.led_brightness, rc_data.led_brightness, rc_data.led_brightness);
        
        led_strip_refresh(led_strip);
        
    } else
    {
        led_strip_clear(led_strip);
    }
    
}

static void configure_led(void)
{
    
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif



void motor_control_task(void *parameters)
{
    
    gpio_set_level(M1_ENN_GPIO, 0);
    gpio_set_level(M3_ENN_GPIO, 0);

    while (1)
    {
        blink_led();
        // 控制电机
        control_gimbal_motor(-(RC_Normalized_data.rightX * 10));
        control_Pitch_motor(RC_Normalized_data.rightY * 5);
        set_motor_direction(RC_Normalized_data.leftX, RC_Normalized_data.leftY);
        vTaskDelay(1);

    }
}

void app_main(void)
{
    static const char *TAG = "USB";
    

    init_usb_device();
    pwm_init();
    configure_led();

    // can_init(void);
    
    // 创建消息队列
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    // xTaskCreate(usb_data_task, "USB task", 4096, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor control task", 8192, NULL, 5, NULL);

    while (1)
    {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (msg.buf_len) {
                memcpy(&rc_data, msg.buf, sizeof(RC_t));
                // ESP_LOG_BUFFER_HEXDUMP(TAG, msg.buf, msg.buf_len, ESP_LOG_INFO);

                RC_Normalized_data.leftX = (rc_data.ch0 - 127.0) / (255.0 - 127.0);
                RC_Normalized_data.leftY = (rc_data.ch1 - 127.0) / (255.0 - 127.0);

                RC_Normalized_data.rightX = (rc_data.ch2 - 127.0) / (255.0 - 127.0);
                RC_Normalized_data.rightY = (rc_data.ch3 - 127.0) / (255.0 - 127.0);
            }
        }
        vTaskDelay(1);
    }
}