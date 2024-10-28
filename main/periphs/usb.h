#include "tinyusb.h"
#include "tusb_cdc_acm.h"

/**
 * @brief Application Queue
 */

typedef struct
{
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; // Data buffer
    size_t buf_len;                                 // Number of bytes received
    uint8_t itf;                                    // Index of CDC device interface
} app_message_t;

typedef struct
{

    // 原始的通道值 (0-255)
    uint8_t ch0;
    uint8_t ch1;
    uint8_t Pitch;

    // uint8_t ch2;
    // uint8_t ch3;

    // uint8_t received_high_byte;
    // uint8_t received_low_byte;

} RC_t;

typedef struct
    {

    // 遥控器操纵杆的规范化值 (-1.0 ~ 1.0)
    float leftX; // 遥控器左手 x 轴的规范化值 (-1 ~ 1)
    float leftY; // 遥控器左手 y 轴的规范化值 (-1 ~ 1)

    float Pitch;

    // float rightX; // 遥控器右手 x 轴的规范化值 (-1 ~ 1)
    // float rightY; // 遥控器右手 y 轴的规范化值 (-1 ~ 1)

} NormalizedRemoteControlData_t;

extern uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
extern QueueHandle_t app_queue;
extern app_message_t msg;

void init_usb_device(void);

// uint16_t fletcher16(uint8_t *data, int count);
