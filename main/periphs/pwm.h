


#ifdef __cplusplus
extern "C" {
#endif


// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (5) // Define the output GPIO
// #define LEDC_DUTY_RES           LEDC_TIMER_8_BIT 
// #define LEDC_DUTY               (4000) 
// #define LEDC_FREQUENCY          (4000) 

// 步进电机1
#define M1_ENN_GPIO GPIO_NUM_4      // 使能引脚 
#define M1_STEP_GPIO GPIO_NUM_1     // 驱动引脚
#define M1_DIR_GPIO GPIO_NUM_2      // 方向引脚

// 步进电机3
#define M3_ENN_GPIO GPIO_NUM_18     // 使能引脚 
#define M3_STEP_GPIO GPIO_NUM_12     // 驱动引脚
#define M3_DIR_GPIO GPIO_NUM_16     // 方向引脚


void pwm_init(void);

void set_motor_direction(float x, float y);

void rotate_to_angle(float target_angle);

void control_gimbal_motor(float angle);
void control_Pitch_motor(float angle);

#ifdef __cplusplus
}
#endif

