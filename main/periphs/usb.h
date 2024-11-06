#include "tinyusb.h"
#include "tinyusb.h"
#include "driver/gpio.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"
/**
 * @brief Application Queue
 */

#define CAMERA_PB_GPIO 10

typedef struct
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; // Data buffer
    size_t buf_len;                                 // Number of bytes received
    uint8_t itf;                                    // Index of CDC device interface
} app_message_t;

typedef struct
{
    uint8_t buf[12];
    size_t buf_len;
} app_send_message_t;

typedef struct
{
    // 原始的通道值 (0-255)
    uint8_t ch0;
    uint8_t ch1;

    uint8_t Pitch;
    uint8_t brightness;
    uint8_t toggle_view;

} RC_t;

typedef struct
{

    float leftX; // 遥控器左手 x 轴的规范化值 (-1 ~ 1)
    float leftY; // 遥控器左手 y 轴的规范化值 (-1 ~ 1)

} NormalizedRemoteControlData_t;

extern uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];


extern app_message_t msg;
extern app_send_message_t send_msg;

extern QueueHandle_t app_queue;        // 接收队列
extern QueueHandle_t send_queue;       // 发送队列



void init_usb_device(void);
void camera_view_direction_init(void);
void set_camera_view(bool toggle_view);
esp_err_t usb_send_data(const uint8_t *data, size_t len);

// uint16_t fletcher16(uint8_t *data, int count);
