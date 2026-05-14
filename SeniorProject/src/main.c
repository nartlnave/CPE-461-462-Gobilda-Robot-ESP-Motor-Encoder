#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

/* encoder GPIO pins */
#define LEFT_ENC_GPIO_A   GPIO_NUM_34
#define LEFT_ENC_GPIO_B   GPIO_NUM_36
#define RIGHT_ENC_GPIO_A  GPIO_NUM_39
#define RIGHT_ENC_GPIO_B  GPIO_NUM_25

/* motor PWM pins */
#define LEFT_MOTOR_PWM    GPIO_NUM_13
#define LEFT_MOTOR_DIR    GPIO_NUM_12
#define RIGHT_MOTOR_PWM   GPIO_NUM_27
#define RIGHT_MOTOR_DIR   GPIO_NUM_33

#define PWM_FREQ_HZ       50
#define PWM_RESOLUTION    LEDC_TIMER_16_BIT
#define PWM_PERIOD_US     20000
#define PWM_MAX_DUTY      ((1 << 16) - 1)

/* goBILDA speed controller accepts about 1100us to 1900us, neutral near 1500us */
#define PULSE_NEUTRAL_US  1500
#define PULSE_MIN_US      1100
#define PULSE_MAX_US      1900

/* control-loop settings */
#define ENCODER_SAMPLE_MS       100
#define PWM_CONTROL_MS          100
#define TARGET_TICKS_PER_SEC    100    // tune this for your desired wheel speed
#define SPEED_TOLERANCE_TPS     10      // acceptable encoder speed error
#define PWM_STEP_PERCENT        2       // how much to increase/decrease when error is too large
#define INITIAL_PWM_PERCENT     20      // starting motor command
#define MAX_PWM_PERCENT         80      // safety cap while testing

static volatile int32_t left_ticks = 0;
static volatile int32_t right_ticks = 0;

volatile int32_t current_left_ticks = 0;
volatile int32_t current_right_ticks = 0;

static portMUX_TYPE tick_mux = portMUX_INITIALIZER_UNLOCKED;

/* updated by encoder_read_task, read by pwm_control_task */
static volatile int32_t left_ticks_per_sec = 0;
static volatile int32_t right_ticks_per_sec = 0;

/* current command sent to the speed controllers */
static int left_pwm_percent = -INITIAL_PWM_PERCENT;   // negative because your original test used left as negative
static int right_pwm_percent = INITIAL_PWM_PERCENT;

static int clamp_int(int value, int min, int max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static void IRAM_ATTR left_encoder_isr(void *arg)
{
    int b_state = gpio_get_level(LEFT_ENC_GPIO_B);

    portENTER_CRITICAL_ISR(&tick_mux);
    if (b_state == 0) {
        left_ticks++;
    } else {
        left_ticks--;
    }
    portEXIT_CRITICAL_ISR(&tick_mux);
}

static void IRAM_ATTR right_encoder_isr(void *arg)
{
    int b_state = gpio_get_level(RIGHT_ENC_GPIO_B);

    portENTER_CRITICAL_ISR(&tick_mux);
    if (b_state == 0) {
        right_ticks++;
    } else {
        right_ticks--;
    }
    portEXIT_CRITICAL_ISR(&tick_mux);
}

static uint32_t us_to_duty(uint32_t pulse_us)
{
    return (pulse_us * PWM_MAX_DUTY) / PWM_PERIOD_US;
}

static uint32_t speed_percent_to_pulse_us(int speed_percent)
{
    speed_percent = clamp_int(speed_percent, -100, 100);

    if (speed_percent >= 0) {
        return PULSE_NEUTRAL_US + ((PULSE_MAX_US - PULSE_NEUTRAL_US) * speed_percent) / 100;
    } else {
        return PULSE_NEUTRAL_US + ((PULSE_NEUTRAL_US - PULSE_MIN_US) * speed_percent) / 100;
    }
}

static void motor_pwm_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t left_pwm_conf = {
        .gpio_num = LEFT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = us_to_duty(PULSE_NEUTRAL_US),
        .hpoint = 0
    };
    ledc_channel_config(&left_pwm_conf);

    ledc_channel_config_t right_pwm_conf = {
        .gpio_num = RIGHT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = us_to_duty(PULSE_NEUTRAL_US),
        .hpoint = 0
    };
    ledc_channel_config(&right_pwm_conf);

    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << LEFT_MOTOR_DIR) | (1ULL << RIGHT_MOTOR_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_conf);

    /* Direction pins are optional if your controller direction is controlled by servo pulse width.
       They are set here so they are not floating. */
    gpio_set_level(LEFT_MOTOR_DIR, 0);
    gpio_set_level(RIGHT_MOTOR_DIR, 0);
}

static void set_left_motor(int speed_percent)
{
    speed_percent = clamp_int(speed_percent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);
    uint32_t pulse_us = speed_percent_to_pulse_us(speed_percent);
    uint32_t duty = us_to_duty(pulse_us);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

static void set_right_motor(int speed_percent)
{
    speed_percent = clamp_int(speed_percent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);
    uint32_t pulse_us = speed_percent_to_pulse_us(speed_percent);
    uint32_t duty = us_to_duty(pulse_us);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

static void encoder_gpio_init(void)
{
    gpio_config_t enc_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << LEFT_ENC_GPIO_A) |
            (1ULL << LEFT_ENC_GPIO_B) |
            (1ULL << RIGHT_ENC_GPIO_A) |
            (1ULL << RIGHT_ENC_GPIO_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&enc_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_ENC_GPIO_A, left_encoder_isr, NULL);
    gpio_isr_handler_add(RIGHT_ENC_GPIO_A, right_encoder_isr, NULL);
}

static void encoder_read_task(void *pvParameters)
{
    int32_t last_left_ticks = 0;
    int32_t last_right_ticks = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_SAMPLE_MS));


        portENTER_CRITICAL(&tick_mux);
        current_left_ticks = left_ticks;
        current_right_ticks = right_ticks;
        portEXIT_CRITICAL(&tick_mux);

        int32_t left_delta = current_left_ticks - last_left_ticks;
        int32_t right_delta = current_right_ticks - last_right_ticks;

        last_left_ticks = current_left_ticks;
        last_right_ticks = current_right_ticks;

        left_ticks_per_sec = (left_delta * 1000) / ENCODER_SAMPLE_MS;
        right_ticks_per_sec = (right_delta * 1000) / ENCODER_SAMPLE_MS;

    }
}

static void pwm_control_task(void *pvParameters)
{
    /* Since the left motor command is negative in your current wiring, its target is negative too. */
    const int32_t target_left_tps = -TARGET_TICKS_PER_SEC;
    const int32_t target_right_tps = TARGET_TICKS_PER_SEC;

    set_left_motor(0);
    set_right_motor(0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        int32_t left_error = target_left_tps - left_ticks_per_sec;
        int32_t right_error = target_right_tps - right_ticks_per_sec;

        /* If measured speed is too low, increase PWM in the direction of the target.
           If measured speed is too high, reduce PWM. */
        if (abs(left_error) > SPEED_TOLERANCE_TPS) {
            if (left_error < 0) {
                left_pwm_percent -= PWM_STEP_PERCENT;
            } else {
                left_pwm_percent += PWM_STEP_PERCENT;
            }
        }

        if (abs(right_error) > SPEED_TOLERANCE_TPS) {
            if (right_error > 0) {
                right_pwm_percent += PWM_STEP_PERCENT;
            } else {
                right_pwm_percent -= PWM_STEP_PERCENT;
            }
        }

        left_pwm_percent = clamp_int(left_pwm_percent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);
        right_pwm_percent = clamp_int(right_pwm_percent, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);

        set_left_motor(left_pwm_percent);
        set_right_motor(right_pwm_percent);

        vTaskDelay(pdMS_TO_TICKS(PWM_CONTROL_MS));
    }
}

static void display_task(void *pvParameters)
{
    while (1)
    {
        printf("\rENC L:%4" PRId32
               " R:%4" PRId32
               " | TPS L:%4" PRId32
               " R:%4" PRId32
               " | PWM L:%3d%%"
               " R:%3d%%     ",
               current_left_ticks,
               current_right_ticks,
               left_ticks_per_sec,
               right_ticks_per_sec,
               left_pwm_percent,
               right_pwm_percent);

        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
void app_main(void)
{
    encoder_gpio_init();
    motor_pwm_init();

    xTaskCreate(encoder_read_task,
                "encoder_read_task",
                4096,
                NULL,
                5,
                NULL);

    xTaskCreate(pwm_control_task,
                "pwm_control_task",
                4096,
                NULL,
                4,
                NULL);

    xTaskCreate(display_task,
                "display_task",
                4096,
                NULL,
                1,
                NULL);
}
