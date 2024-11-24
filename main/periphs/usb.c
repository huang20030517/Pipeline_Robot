#include "periphs/usb.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"

/// @brief 错误检查并打印日志
#define ESP_ERROR_CHECK_WITH_LOG(x)                                                                                          \
    do                                                                                                                       \
    {                                                                                                                        \
        esp_err_t __err_rc = (x);                                                                                            \
        if (__err_rc != ESP_OK)                                                                                              \
        {                                                                                                                    \
            ESP_LOGE("USB", "ESP_ERROR at %s:%d, code: 0x%x (%s)", __FILE__, __LINE__, __err_rc, esp_err_to_name(__err_rc)); \
        }                                                                                                                    \
    } while (0)

static const char *TAG = "USB";
uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; ///< CDC 数据接收缓冲区

extern QueueHandle_t app_queue; ///< 全局队列句柄
extern app_message_t msg;       ///< 全局消息结构

/**
 * @brief CDC 设备数据接收回调
 * @param[in] itf   CDC 设备索引
 * @param[in] event CDC 事件类型
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;

    // 从 CDC 设备读取数据
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK)
    {
        app_message_t tx_msg = {
            .buf_len = rx_size,
            .itf = itf,
        };

        memcpy(tx_msg.buf, rx_buf, rx_size);
        xQueueSend(app_queue, &tx_msg, 0);
    }
    else
    {
        ESP_LOGE(TAG, "Read Error");
    }
}

/**
 * @brief CDC 设备线路状态变化回调
 * @param[in] itf   CDC 设备索引
 * @param[in] event CDC 事件类型
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Interface %d Line State Changed: DTR=%d, RTS=%d", itf, dtr, rts);
}

/**
 * @brief 初始化 USB 设备
 */
void init_usb_device(void)
{
    ESP_LOGI(TAG, "Initializing USB device...");

    // 设备描述符
    static const tusb_desc_device_t device_descriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = 0x1234,
        .idProduct = 0x5678,
        .bcdDevice = 0x0100,
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01,
    };

    // USB 配置
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
#endif
    };

    // 安装 USB 驱动程序
    ESP_ERROR_CHECK_WITH_LOG(tinyusb_driver_install(&tusb_cfg));

    // 配置 CDC-ACM
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL,
    };

    // 初始化第一个 CDC 实例
    ESP_ERROR_CHECK_WITH_LOG(tusb_cdc_acm_init(&acm_cfg));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    // 如果启用了多个 CDC 实例，初始化第二个实例
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK_WITH_LOG(tusb_cdc_acm_init(&acm_cfg));
#endif

    ESP_LOGI(TAG, "USB device initialized.");
}

/**
 * @brief 初始化摄像头方向控制
 */
void camera_view_direction_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CAMERA_PB_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    ESP_ERROR_CHECK_WITH_LOG(gpio_config(&io_conf));
    gpio_set_level(CAMERA_PB_GPIO, 1);
}

/**
 * @brief 设置摄像头方向
 * @param toggle_view 切换方向的布尔值
 */
void set_camera_view(bool toggle_view)
{
    static uint8_t v = 0;

    if (v != toggle_view)
    {
        gpio_set_level(CAMERA_PB_GPIO, toggle_view);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CAMERA_PB_GPIO, 1);
    }

    v = toggle_view;
}

/**
 * @brief 发送数据到 USB CDC
 * @param[in] data 数据缓冲区指针
 * @param[in] len  数据长度
 * @return esp_err_t 返回操作结果
 */
esp_err_t usb_send_data(const uint8_t *data, size_t len)
{
    size_t bytes_queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len);
    if (bytes_queued != len)
    {
        return ESP_FAIL;
    }

    return tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, portMAX_DELAY);
}
