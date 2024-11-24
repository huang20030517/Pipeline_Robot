#include "periphs/encoder.h"

/// @brief 初始化编码器的 GPIO 和 PCNT 外设
void encoder_init(void)
{
    // 启用 GPIO 的上拉电阻
    gpio_pullup_en(ENCODER1_A_GPIO);
    gpio_pullup_en(ENCODER1_B_GPIO);
    gpio_pullup_en(ENCODER2_A_GPIO);
    gpio_pullup_en(ENCODER2_B_GPIO);

    // 配置第一个编码器
    pcnt_config_t pcnt_config0 = {
        .pulse_gpio_num = ENCODER1_A_GPIO, // 编码器脉冲信号
        .ctrl_gpio_num = ENCODER1_B_GPIO,  // 编码器方向信号
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,         // 脉冲正转时计数增加
        .neg_mode = PCNT_COUNT_DEC,         // 脉冲反转时计数减少
        .lctrl_mode = PCNT_MODE_REVERSE,    // 方向信号低时反转计数
        .hctrl_mode = PCNT_MODE_KEEP,       // 方向信号高时保持方向
        .counter_h_lim = COUNTER_MAX_LIMIT, // 计数上限
        .counter_l_lim = COUNTER_MIN_LIMIT, // 计数下限
    };
    pcnt_unit_config(&pcnt_config0);

    // 配置第二个编码器
    pcnt_config_t pcnt_config1 = {
        .pulse_gpio_num = ENCODER2_A_GPIO,
        .ctrl_gpio_num = ENCODER2_B_GPIO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_1,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = COUNTER_MAX_LIMIT,
        .counter_l_lim = COUNTER_MIN_LIMIT,
    };
    pcnt_unit_config(&pcnt_config1);

    // 配置滤波器，滤除高频干扰信号
    pcnt_set_filter_value(PCNT_UNIT_0, 200); // 设置滤波器值为 200
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_set_filter_value(PCNT_UNIT_1, 200);
    pcnt_filter_enable(PCNT_UNIT_1);

    // 启动 PCNT 计数器
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}
