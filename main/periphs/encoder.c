#include "periphs/encoder.h"

void encoder_init(void)
{

    gpio_pullup_en(ENCODER1_A_GPIO);
    gpio_pullup_en(ENCODER1_B_GPIO);

    gpio_pullup_en(ENCODER2_A_GPIO);
    gpio_pullup_en(ENCODER2_B_GPIO);

    // 配置第一个编码器的PCNT单元
    pcnt_config_t pcnt_config0 = {
        .pulse_gpio_num = ENCODER1_A_GPIO,
        .ctrl_gpio_num = ENCODER1_B_GPIO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC, // 正转脉冲增加计数
        .neg_mode = PCNT_COUNT_DEC, // 反转脉冲减少计数
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = COUNTER_MAX_LIMIT, // 设置计数上限为每转脉冲数
        .counter_l_lim = COUNTER_MIN_LIMIT, // 设置计数下限为负的每转脉冲数
    };
    pcnt_unit_config(&pcnt_config0);

    pcnt_config_t pcnt_config1 = {
        .pulse_gpio_num = ENCODER2_A_GPIO,
        .ctrl_gpio_num = ENCODER2_B_GPIO,
        .channel = PCNT_CHANNEL2,
        .unit = PCNT_UNIT_1,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = COUNTER_MAX_LIMIT, // 设置计数上限为每转脉冲数
        .counter_l_lim = COUNTER_MIN_LIMIT, // 设置计数下限为负的每转脉冲数
    };
    pcnt_unit_config(&pcnt_config1);

    // 设置并启用滤波器
    pcnt_set_filter_value(PCNT_UNIT_0, 200); // 增加滤波器值
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_set_filter_value(PCNT_UNIT_1, 200); // 增加滤波器值
    pcnt_filter_enable(PCNT_UNIT_1);

    // 启动 PCNT 计数器
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}