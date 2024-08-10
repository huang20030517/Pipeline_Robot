
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define COUNTER_MAX_LIMIT 14420  // 设置为两圈的脉冲数
#define COUNTER_MIN_LIMIT -14420 // 设置为负两圈的脉冲数

    void encoder_init(void);

#ifdef __cplusplus
}
#endif
