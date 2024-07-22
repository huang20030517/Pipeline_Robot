#include "periphs/can.h"
#include "esp_log.h"

static const char* TAG = "CAN_EXAMPLE";



void can_init(void)
{
     // 配置 TWAI 驱动
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_2,
        .rx_io = GPIO_NUM_1,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 50,
        .rx_queue_len = 50,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
    };

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // 使用 1Mbps 的波特率
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 初始化 TWAI 驱动
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %d", ret);
        // return 1;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %d", ret);
        twai_driver_uninstall();
        // return 1;
    }
}