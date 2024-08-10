/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "stdint.h"
#include "stdio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

#include "periphs/mpu6050_driver.c"
#include "periphs/usb.h"
#include "periphs/pwm.h"
#include "periphs/encoder.h"


NormalizedRemoteControlData_t RC_Normalized_data = {0, 0, 0};
RC_t rc_data = {127, 127, 20};

void mpu6050_read_task(void *parameters)
{

    // i2c_sensor_mpu6050_init();

    while(1)
    {

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void encoder_read_task(void *parameters)
{
    encoder_init();

    // 初始化计数器和速度变量
    int16_t count_prev[2] = {0, 0};
    int16_t count[2] = {0, 0};
    int16_t delta[2] = {0, 0};
    float linear_speed[2] = {0, 0};

    while (1)
    {
        // 获取计数器值
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);

        for (int i = 0; i < 2; ++i)
        {
            delta[i] = count[i] - count_prev[i];

            // 处理计数器回绕情况
            if (delta[i] > (COUNTER_MAX_LIMIT / 2))
            {
                delta[i] -= COUNTER_MAX_LIMIT;
            }
            else if (delta[i] < -(COUNTER_MAX_LIMIT / 2))
            { 
                delta[i] += COUNTER_MAX_LIMIT;
            }    
            count_prev[i] = count[i];

            // 计算转速 (RPM)
            float speed = (delta[i] * 60.0f) / 14420.0f;

            // 计算线速度 (cm/s)
            linear_speed[i] = speed * 74.45592f / 60.0f;
            // 打印速度和线速度
            // printf("Encoder %d - RPM: %.2f, Linear Speed: %.2f cm/s\n", i, speed, linear_speed[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void motor_control_task(void *parameters)
{
    pwm_init();

    while (1)
    {

        set_servo_angle(RC_Normalized_data.Pitch);
        set_motor_direction(RC_Normalized_data.leftX, RC_Normalized_data.leftY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    init_usb_device();

    // 创建消息队列
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    // xTaskCreate(usb_data_task, "USB task", 4096, NULL, 5, NULL);

    xTaskCreate(motor_control_task, "motor control task", 8192, NULL, 5, NULL);
    xTaskCreate(encoder_read_task, "encoder read task", 4096, NULL, 4, NULL);
    xTaskCreate(mpu6050_read_task, "mpu6050 read task", 8192, NULL, 4, NULL);
    
    while (1)
    {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
        {
            if (msg.buf_len)
            {
                memcpy(&rc_data, msg.buf, sizeof(RC_t));
                // ESP_LOG_BUFFER_HEXDUMP("usb", msg.buf, msg.buf_len, ESP_LOG_INFO);

                // 归一化处理
                RC_Normalized_data.leftX = (rc_data.ch0 - 127.0) / (254.0 - 127.0);
                RC_Normalized_data.leftY = (rc_data.ch1 - 127.0) / (254.0 - 127.0);
                RC_Normalized_data.Pitch = rc_data.Pitch;

                // printf("x: %.2f     y: %.2f     Pitch: %d\r\n", RC_Normalized_data.leftX, RC_Normalized_data.leftY, rc_data.Pitch);
            }
        }
        vTaskDelay(1);
    }
}