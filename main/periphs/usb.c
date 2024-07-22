#include "periphs/usb.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"



uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
QueueHandle_t app_queue;
app_message_t msg;

/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    
    static const char *TAG = "USB";

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {

        app_message_t tx_msg = {
            .buf_len = rx_size,
            .itf = itf,
        };

        memcpy(tx_msg.buf, rx_buf, rx_size);
        xQueueSend(app_queue, &tx_msg, 0);
    } else {
        ESP_LOGE(TAG, "Read Error");
    }
}

/**
 * @brief CDC device line change callback
 *
 * CDC device signals, that the DTR, RTS states changed
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    static const char *TAG = "USB";

    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "通道 %d 的线路状态发生变化: DTR:%d, RTS:%d", itf, dtr, rts);
}

void init_usb_device(void)
{

    static const char *TAG = "USB";

    ESP_LOGI(TAG, "正在执行 USB 设备的初始化操作");

    static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t), // 描述符长度
    .bDescriptorType    = TUSB_DESC_DEVICE, // 描述符类型为设备描述符
    .bcdUSB             = 0x0200, // USB 规范版本 2.0
    .bDeviceClass       = 0x00,   // 使用接口描述符中的类别信息
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol     = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE, // 端点 0 的最大包大小
    
    .idVendor           = 0x1234, // 厂商 ID
    .idProduct          = 0x5678, // 产品 ID
    
    .bcdDevice          = 0x0100, // 设备版本号 1.00
    .iManufacturer      = 0x01,   // 制造商字符串描述符的索引号
    .iProduct           = 0x02,   // 产品字符串描述符的索引号
    .iSerialNumber      = 0x03,   // 序列号字符串描述符的索引号
    .bNumConfigurations = 0x01    // 仅有一个配置
}; // 4660 22136 2002

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &device_descriptor,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    // 安装和初始化 USB 设备驱动程序
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // 配置 CDC-ACM
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    // 初始化 CDC-ACM 功能
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    // 判断是否配置了多个 CDC-ACM 虚拟串口
#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif  

    ESP_LOGI(TAG, "USB 初始化已经完成");
}


// uint16_t fletcher16(uint8_t *data, int count) {
//     uint16_t sum1 = 0;
//     uint16_t sum2 = 0;

//     for (int i = 0; i < count; i++) {
//         sum1 = (sum1 + data[i]) % 255;
//         sum2 = (sum2 + sum1) % 255;
//     }

//     return (sum2 << 8) | sum1;
// }

