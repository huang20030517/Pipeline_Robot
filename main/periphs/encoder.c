#include "periphs/encoder.h"
#include "driver/gpio.h"
#include "stdio.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

// // 编码器1引脚定义
// #define ENCODER1_A_GPIO GPIO_NUM_35
// #define ENCODER1_B_GPIO GPIO_NUM_36

// // 编码器2引脚定义
// #define ENCODER2_A_GPIO GPIO_NUM_37
// #define ENCODER2_B_GPIO GPIO_NUM_38

// #define EXAMPLE_PCNT_HIGH_LIMIT 100
// #define EXAMPLE_PCNT_LOW_LIMIT  -100

static const char *TAG = "example";

// void example_pcnt_on_reach(pcnt_unit_handle_t pcnt_unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
// {
//     int value;
//     if (xQueueSendFromISR(user_ctx, &edata->watch_point_value, NULL) != pdTRUE) {
//     }
// }

void encoder_init(void)
{
    // 初始化两个PCNT(脉冲计数)单元:
//    pcnt_unit_config_t unit_config = {
//     .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
//     .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
//     };
//     pcnt_unit_handle_t pcnt_unit_1 = NULL, pcnt_unit_2 = NULL;
//     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_1));
//     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_2));

//     // 配置两个编码器的通道:
//     pcnt_chan_config_t chan_a_config = {
//     .edge_gpio_num = ENCODER1_A_GPIO,
//     .level_gpio_num = ENCODER1_B_GPIO,
//     };
//     pcnt_channel_handle_t pcnt_chan_a = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &chan_a_config, &pcnt_chan_a));
//     pcnt_chan_config_t chan_b_config = {
//         .edge_gpio_num = ENCODER1_B_GPIO,
//         .level_gpio_num = ENCODER1_A_GPIO,
//     };
//     pcnt_channel_handle_t pcnt_chan_b = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &chan_b_config, &pcnt_chan_b));

//     // 配置编码器2的通道
//     pcnt_chan_config_t chan_c_config = {
//         .edge_gpio_num = ENCODER2_A_GPIO,
//         .level_gpio_num = ENCODER2_B_GPIO,
//     };
//     pcnt_channel_handle_t pcnt_chan_c = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &chan_c_config, &pcnt_chan_c));
//     pcnt_chan_config_t chan_d_config = {
//         .edge_gpio_num = ENCODER2_B_GPIO,
//         .level_gpio_num = ENCODER2_A_GPIO,
//     };
//     pcnt_channel_handle_t pcnt_chan_d = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &chan_d_config, &pcnt_chan_d));

//     // 设置两个编码器的边沿和电平动作:
//     // 设置编码器1的动作
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

//     // 设置编码器2的动作
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_c, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_c, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_d, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_d, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

//     // 添加监视点和注册回调函数
//     ESP_LOGI(TAG, "add watch points and register callbacks");
//     int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
//     for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
//         ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_1, watch_points[i]));
//         ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_2, watch_points[i]));
//     }

//        // 为两个PCNT单元注册事件回调函数
//     pcnt_event_callbacks_t cbs = {
//         .on_reach = example_pcnt_on_reach,
//     };
//     QueueHandle_t queue = xQueueCreate(10, sizeof(int));
//     ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_1, &cbs, queue));
//     ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_2, &cbs, queue));

//     // 启用、清零并启动两个PCNT单元
//     ESP_LOGI(TAG, "enable pcnt units");
//     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_1));
//     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_2));

//     ESP_LOGI(TAG, "clear pcnt units");
//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_1));
//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_2));

//     ESP_LOGI(TAG, "start pcnt units");
//     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_1));
//     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_2));
} 
