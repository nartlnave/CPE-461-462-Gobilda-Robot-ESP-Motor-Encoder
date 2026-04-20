#include <stdio.h>
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

/* test PWM pins */
#define LEFT_MOTOR_PWM    GPIO_NUM_13
#define LEFT_MOTOR_DIR    GPIO_NUM_12
#define RIGHT_MOTOR_PWM   GPIO_NUM_27
#define RIGHT_MOTOR_DIR   GPIO_NUM_33

#define PWM_FREQ_HZ       20000   // 20 kHz
#define PWM_RESOLUTION    LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY      ((1 << 10) - 1)   // 1023 for 10-bit

static volatile int32_t left_ticks = 0;
static volatile int32_t right_ticks = 0;

static void IRAM_ATTR left_encoder_isr(void *arg) {
    int b_state = gpio_get_level(LEFT_ENC_GPIO_B);
    if (b_state == 0) {
        left_ticks--;
    } else {
        left_ticks++;
    }
}

static void IRAM_ATTR right_encoder_isr(void *arg) {
    int b_state = gpio_get_level(RIGHT_ENC_GPIO_B);
    if (b_state == 0) {
        right_ticks++;
    } else {
        right_ticks--;
    }
}

static void motor_pwm_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Left motor PWM channel
    ledc_channel_config_t left_pwm_conf = {
        .gpio_num   = LEFT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&left_pwm_conf);

    // Right motor PWM channel
    ledc_channel_config_t right_pwm_conf = {
        .gpio_num   = RIGHT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&right_pwm_conf);

    // Direction pins
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << LEFT_MOTOR_DIR) | (1ULL << RIGHT_MOTOR_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_conf);
}

static void set_left_motor(int speed_percent)
{
    // speed_percent: -100 to +100
    if (speed_percent >= 0) {
        gpio_set_level(LEFT_MOTOR_DIR, 1);
    } else {
        gpio_set_level(LEFT_MOTOR_DIR, 0);
        speed_percent = -speed_percent;
    }

    if (speed_percent > 100) speed_percent = 100;

    uint32_t duty = (speed_percent * PWM_MAX_DUTY) / 100;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

static void set_right_motor(int speed_percent)
{
    if (speed_percent >= 0) {
        gpio_set_level(RIGHT_MOTOR_DIR, 1);
    } else {
        gpio_set_level(RIGHT_MOTOR_DIR, 0);
        speed_percent = -speed_percent;
    }

    if (speed_percent > 100) speed_percent = 100;

    uint32_t duty = (speed_percent * PWM_MAX_DUTY) / 100;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

void app_main(void)
{
    gpio_config_t enc_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << LEFT_ENC_GPIO_A)  |
            (1ULL << LEFT_ENC_GPIO_B)  |
            (1ULL << RIGHT_ENC_GPIO_A) |
            (1ULL << RIGHT_ENC_GPIO_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&enc_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_ENC_GPIO_A, left_encoder_isr, NULL);
    gpio_isr_handler_add(RIGHT_ENC_GPIO_A, right_encoder_isr, NULL);

    motor_pwm_init();

    // Example: both motors forward at 50%
    set_left_motor(100);
    set_right_motor(100);

    while (1) {
        // printf("Left ticks: %" PRId32 " | Right ticks: %" PRId32 "\n",
        //        left_ticks, right_ticks);

        // vTaskDelay(pdMS_TO_TICKS(250));


        // PWM LED TEST
        // Left PWM (GPIO 13) is also set to the ESP LED, will be used to test PWM output
        for (int i = 0; i < 100; i++)
        {
            set_left_motor(i);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        for (int i = 100; i > 0; i--)
        {
            set_left_motor(i);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        for (int i = 0; i < 100; i++)
        {
            set_left_motor(i);
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (int i = 100; i > 0; i--)
        {
            set_left_motor(i);
            vTaskDelay(pdMS_TO_TICKS(20));
        }


    }
}