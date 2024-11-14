#include "stdint.h"
#include "stdio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "periphs/mpu6050_driver.c"
#include "periphs/encoder.h"
#include "periphs/usb.h"
#include "periphs/pwm.h"

QueueHandle_t app_queue;
QueueHandle_t send_queue;

NormalizedRemoteControlData_t RC_Normalized_data = {0, 0};
RC_t rc_data = {127, 127, 90, 0, 1};

mpu6050_temp_value_t temp;
complimentary_angle_t angle;

app_message_t msg;
app_send_message_t send_msg;

float speed = 0;

void mpu6050_read_task(void *parameters)
{
    // static const char *TAG = "mpu6050";
    // esp_err_t ret;
    // uint8_t mpu6050_deviceid;
    i2c_sensor_mpu6050_init();

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    complimentary_angle_t raw_angle;

    angle.pitch = 0.0f;
    angle.roll = 0.0f;
    angle.yaw = 0.0f;

    while (1)
    {
        // // 验证 MPU6050 传感器是否正确连接并工作
        // ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
        // TEST_ASSERT_EQUAL(ESP_OK, ret);
        // TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Who Am I register does not contain expected data");

        // 读取数据
        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);
        mpu6050_get_temp(mpu6050, &temp);

        if (RC_Normalized_data.leftX == 0 && RC_Normalized_data.leftY == 0)
        {
            angle.pitch = 0.0f;
            angle.roll = 0.0f;
            angle.yaw = 0.0f;
        }
        else
        {
            mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angle);
        }

        // 测试
        // raw_angle.roll = atan2(acce.acce_y, acce.acce_z) * 57.2958f;
        // raw_angle.pitch = atan2(acce.acce_x, acce.acce_z) * 57.2958f;
        // raw_angle.yaw += gyro.gyro_z * 0.1f; // 使用简单积分模拟 yaw 计算

        // printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", raw_angle.pitch, raw_angle.roll, raw_angle.yaw, angle.pitch, angle.roll, angle.yaw);
        // printf("%.2f,%.2f,%.2f\r\n", temp.temp, raw_angle.pitch, angle.pitch);
        printf("%.2f,%.2f,%.2f\r\n", angle.pitch, angle.roll, angle.yaw);
        // 打包数据并发送到 send_queue
        memset(send_msg.buf, 0, sizeof(send_msg.buf));
        memcpy(&send_msg.buf[0], &temp.temp, sizeof(float));
        memcpy(&send_msg.buf[4], &angle.yaw, sizeof(float));
        memcpy(&send_msg.buf[8], &angle.roll, sizeof(float));
        memcpy(&send_msg.buf[12], &angle.pitch, sizeof(float));

        memcpy(&send_msg.buf[16], &speed, sizeof(float));

        send_msg.buf_len = 20;
        xQueueSend(send_queue, &send_msg, 0);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void encoder_read_task(void *parameters)
{
    encoder_init();
    int16_t count_prev[2] = {0, 0};
    int16_t count[2] = {0, 0};
    int16_t delta[2] = {0, 0};
    float total_distance[2] = {0, 0};

    float distance_per_pulse = (3.14159f * WHEEL_DIAMETER) / (COUNTER_MAX_LIMIT / 2); // 单位：厘米
    while (1)
    {
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);

        // 处理计数回绕;
        for (int i = 0; i < 2; ++i)
        {
            delta[i] = count[i] - count_prev[i];
            if (delta[i] > (COUNTER_MAX_LIMIT / 2))
                delta[i] -= COUNTER_MAX_LIMIT;
            else if (delta[i] < -(COUNTER_MAX_LIMIT / 2))
                delta[i] += COUNTER_MAX_LIMIT;

            count_prev[i] = count[i];

            total_distance[i] = delta[i] * distance_per_pulse;
        }

        speed = (total_distance[0] + total_distance[1]) / 2;

        // printf("%.2f    %.2f\n", total_distance[0], total_distance[1]);
        // printf("speed %.2f\n", speed);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void motor_control_task(void *parameters)
{
    pwm_init();
    camera_view_direction_init();

    rc_data.Pitch = 90;

    while (1)
    {
        set_motor_direction(RC_Normalized_data.leftX, RC_Normalized_data.leftY);
        set_servo_angle(rc_data.Pitch);
        set_light_brightness(rc_data.brightness);
        set_camera_view(rc_data.toggle_view);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void usb_send_data_task(void *parameters)
{
    while (1)
    {

        if (xQueueReceive(send_queue, &send_msg, portMAX_DELAY))
        {
            if (send_msg.buf_len > 0)
            {
                // ESP_LOG_BUFFER_HEXDUMP("USB_SEND", send_msg.buf, send_msg.buf_len, ESP_LOG_INFO);
                usb_send_data(send_msg.buf, send_msg.buf_len);

                // ESP_LOGI("USB", "Data sent to USB: %d bytes", send_msg.buf_len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    init_usb_device();

    // 创建消息队列
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    send_queue = xQueueCreate(5, sizeof(app_send_message_t));
    assert(send_queue);

    RC_Normalized_data.leftX = 0;
    RC_Normalized_data.leftY = 0;

    xTaskCreate(motor_control_task, "motor control task", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_read_task, "encoder read task", 4096, NULL, 3, NULL);
    xTaskCreate(mpu6050_read_task, "mpu6050 read task", 8192, NULL, 3, NULL);
    xTaskCreate(usb_send_data_task, "usb send data task", 8192, NULL, 5, NULL);
    while (1)
    {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
        {
            if (msg.buf_len)
            {
                memcpy(&rc_data, msg.buf, sizeof(RC_t));

                // 归一化处理
                RC_Normalized_data.leftX = (rc_data.ch0 - 127.0) / (254.0 - 127.0);
                RC_Normalized_data.leftY = (rc_data.ch1 - 127.0) / (254.0 - 127.0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}