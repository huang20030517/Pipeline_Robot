#ifndef ENCODER_H
#define ENCODER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// @brief 编码器参数
#define WHEEL_DIAMETER 2.3f ///< 车轮直径（单位：厘米）

// 减速比选择，根据使用的电机调整 COUNTER_MAX_LIMIT 和 COUNTER_MIN_LIMIT
#ifdef REDUCTION_RATIO_1030
#define COUNTER_MAX_LIMIT 14420  ///< 减速比 1030：两圈脉冲计数上限
#define COUNTER_MIN_LIMIT -14420 ///< 减速比 1030：两圈脉冲计数下限
#else
#define COUNTER_MAX_LIMIT 8442  ///< 减速比 603：两圈脉冲计数上限
#define COUNTER_MIN_LIMIT -8442 ///< 减速比 603：两圈脉冲计数下限
#endif

/// @brief 编码器 GPIO 引脚定义
#define ENCODER1_A_GPIO 4 ///< 编码器 1 A 相信号 GPIO 引脚
#define ENCODER1_B_GPIO 3 ///< 编码器 1 B 相信号 GPIO 引脚
#define ENCODER2_A_GPIO 5 ///< 编码器 2 A 相信号 GPIO 引脚
#define ENCODER2_B_GPIO 6 ///< 编码器 2 B 相信号 GPIO 引脚

/// @brief PCNT（脉冲计数器）通道配置
#define PCNT_CHANNEL1 PCNT_CHANNEL_0 ///< 编码器 1 使用的 PCNT 通道
#define PCNT_CHANNEL2 PCNT_CHANNEL_1 ///< 编码器 2 使用的 PCNT 通道

/// @brief 初始化编码器的 GPIO 和 PCNT 配置
void encoder_init(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
