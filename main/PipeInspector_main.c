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

// 定义常量
#define APP_QUEUE_SIZE 5
#define SEND_QUEUE_SIZE 5

// 定义任务栈大小
#define MPU6050_TASK_STACK 8192
#define ENCODER_TASK_STACK 4096
#define MOTOR_TASK_STACK 4096
#define USB_TASK_STACK 8192
#define QUEUE_RECEIVE_TASK_STACK 4096

// 定义任务优先级
#define MOTOR_TASK_PRIORITY 5
#define ENCODER_TASK_PRIORITY 3
#define MPU6050_TASK_PRIORITY 3
#define USB_TASK_PRIORITY 5
#define QUEUE_RECEIVE_TASK_PRIORITY 4

QueueHandle_t app_queue;
QueueHandle_t send_queue;

NormalizedRemoteControlData_t RC_Normalized_data = {0, 0};
RC_t rc_data = {127, 127, 90, 0, 1};
mpu6050_temp_value_t temp;
complimentary_angle_t angle = {0.0f, 0.0f, 0.0f};
app_message_t msg;
app_send_message_t send_msg;
float speed = 0.0f;

/// @brief 读取 MPU6050 传感器数据并更新角度信息
/// @param parameters FreeRTOS 任务传递参数（未使用）
void mpu6050_read_task(void *parameters)
{
    i2c_sensor_mpu6050_init();
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    while (1)
    {
        mpu6050_get_acce(mpu6050, &acce);
        mpu6050_get_gyro(mpu6050, &gyro);
        mpu6050_get_temp(mpu6050, &temp);

        if (RC_Normalized_data.leftX == 0 && RC_Normalized_data.leftY == 0)
        {
            angle.pitch = angle.roll = angle.yaw = 0.0f;
        }
        else
        {
            mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &angle);
        }

        float data[5] = {temp.temp, angle.yaw, angle.roll, angle.pitch, speed};
        memcpy(send_msg.buf, data, sizeof(data));
        send_msg.buf_len = sizeof(data);
        xQueueSend(send_queue, &send_msg, 0);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/// @brief 读取编码器数据并计算速度
/// @param parameters FreeRTOS 任务传递参数（未使用）
void encoder_read_task(void *parameters)
{
    encoder_init();
    int16_t count_prev[2] = {0, 0};
    int16_t count[2] = {0, 0};
    float total_distance[2] = {0, 0};
    float distance_per_pulse = (3.14159f * WHEEL_DIAMETER) / (COUNTER_MAX_LIMIT / 2);

    while (1)
    {
        pcnt_get_counter_value(PCNT_UNIT_0, &count[0]);
        pcnt_get_counter_value(PCNT_UNIT_1, &count[1]);

        for (int i = 0; i < 2; ++i)
        {
            int16_t delta = count[i] - count_prev[i];
            if (delta > COUNTER_MAX_LIMIT / 2)
                delta -= COUNTER_MAX_LIMIT;
            else if (delta < -COUNTER_MAX_LIMIT / 2)
                delta += COUNTER_MAX_LIMIT;

            count_prev[i] = count[i];
            total_distance[i] = delta * distance_per_pulse;
        }

        speed = (total_distance[0] + total_distance[1]) / 2;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/// @brief 控制电机、舵机及相关外设
/// @param parameters FreeRTOS 任务传递参数（未使用）
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

/// @brief 通过 USB 发送数据
/// @param parameters FreeRTOS 任务传递参数（未使用）
void usb_send_data_task(void *parameters)
{
    while (1)
    {
        if (xQueueReceive(send_queue, &send_msg, portMAX_DELAY))
        {
            if (send_msg.buf_len > 0)
            {
                usb_send_data(send_msg.buf, send_msg.buf_len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/// @brief 接收队列消息并更新 RC 数据
/// @param parameters FreeRTOS 任务传递参数（未使用）
void queue_receive_task(void *parameters)
{
    app_message_t msg;

    while (1)
    {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
        {
            if (msg.buf_len)
            {
                memcpy(&rc_data, msg.buf, sizeof(RC_t));
                RC_Normalized_data.leftX = (rc_data.ch0 - 127.0f) / 127.0f;
                RC_Normalized_data.leftY = (rc_data.ch1 - 127.0f) / 127.0f;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/// @brief 应用程序主入口
void app_main(void)
{
    init_usb_device();

    // 创建消息队列
    app_queue = xQueueCreate(APP_QUEUE_SIZE, sizeof(app_message_t));
    assert(app_queue);
    send_queue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(app_send_message_t));
    assert(send_queue);

    RC_Normalized_data.leftX = RC_Normalized_data.leftY = 0;

    // 创建任务
    xTaskCreate(motor_control_task, "Motor Control Task", MOTOR_TASK_STACK, NULL, MOTOR_TASK_PRIORITY, NULL);
    xTaskCreate(encoder_read_task, "Encoder Read Task", ENCODER_TASK_STACK, NULL, ENCODER_TASK_PRIORITY, NULL);
    xTaskCreate(mpu6050_read_task, "MPU6050 Read Task", MPU6050_TASK_STACK, NULL, MPU6050_TASK_PRIORITY, NULL);
    xTaskCreate(usb_send_data_task, "USB Send Data Task", USB_TASK_STACK, NULL, USB_TASK_PRIORITY, NULL);
    xTaskCreate(queue_receive_task, "Queue Receive Task", QUEUE_RECEIVE_TASK_STACK, NULL, QUEUE_RECEIVE_TASK_PRIORITY, NULL);

    while (1)
    {
        vTaskDelay(portMAX_DELAY); // 无限阻塞，直到系统需要唤醒
    }
}
