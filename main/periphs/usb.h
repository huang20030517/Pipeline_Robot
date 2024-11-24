#ifndef USB_DRIVER_H
#define USB_DRIVER_H

#include "tinyusb.h"
#include "driver/gpio.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"

/**
 * @brief 摄像头 GPIO 配置
 */
#define CAMERA_PB_GPIO 10 ///< 摄像头方向切换的 GPIO 引脚

/**
 * @brief 应用程序消息结构，用于接收数据
 */
typedef struct
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; ///< 数据缓冲区
    size_t buf_len;                                 ///< 接收到的数据字节数
    uint8_t itf;                                    ///< CDC 接口索引
} app_message_t;

/**
 * @brief 应用程序消息结构，用于发送数据
 */
typedef struct
{
    uint8_t buf[20]; ///< 数据缓冲区，固定大小 20 字节
    size_t buf_len;  ///< 要发送的数据字节数
} app_send_message_t;

/**
 * @brief 遥控器原始数据结构
 */
typedef struct
{
    uint8_t ch0;        ///< 通道 0 的原始值 (0-255)
    uint8_t ch1;        ///< 通道 1 的原始值 (0-255)
    uint8_t Pitch;      ///< 舵机角度值
    uint8_t brightness; ///< 灯光亮度值
    uint8_t toggle_view;///< 摄像头视角切换标志
} RC_t;

/**
 * @brief 遥控器规范化数据结构
 */
typedef struct
{
    float leftX; ///< 遥控器左手 X 轴规范化值 (-1.0 ~ 1.0)
    float leftY; ///< 遥控器左手 Y 轴规范化值 (-1.0 ~ 1.0)
} NormalizedRemoteControlData_t;

// 外部全局变量声明
extern uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; ///< CDC 数据接收缓冲区
extern app_message_t msg;                                 ///< 接收消息
extern app_send_message_t send_msg;                      ///< 发送消息
extern QueueHandle_t app_queue;                          ///< 接收队列句柄
extern QueueHandle_t send_queue;                         ///< 发送队列句柄

// 函数声明
/**
 * @brief 初始化 USB 设备
 */
void init_usb_device(void);

/**
 * @brief 初始化摄像头方向控制 GPIO
 */
void camera_view_direction_init(void);

/**
 * @brief 设置摄像头视角切换
 * @param toggle_view 摄像头视角切换标志（true 或 false）
 */
void set_camera_view(bool toggle_view);

/**
 * @brief 发送数据到 USB CDC
 * @param[in] data 数据缓冲区指针
 * @param[in] len 数据长度
 * @return esp_err_t 返回操作结果
 */
esp_err_t usb_send_data(const uint8_t *data, size_t len);

#endif // USB_DRIVER_H
