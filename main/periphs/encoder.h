
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define COUNTER_MAX_LIMIT 14420  // 两圈的脉冲数
#define COUNTER_MIN_LIMIT -14420 // 负两圈的脉冲数

// 减速比: 1030
// 脉冲每圈 (PPR): 7

// 编码器 GPIO 引脚定义
#define ENCODER1_A_GPIO 3
#define ENCODER1_B_GPIO 4
#define ENCODER2_A_GPIO 5
#define ENCODER2_B_GPIO 6

// PCNT（脉冲计数器）配置
#define PCNT_CHANNEL1 PCNT_CHANNEL_0
#define PCNT_CHANNEL2 PCNT_CHANNEL_1

    void encoder_init(void);

#ifdef __cplusplus
}
#endif
