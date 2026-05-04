#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"

/* encoder GPIO pins */
#define LEFT_ENC_GPIO_A GPIO_NUM_34 // GPIO 34 is input-only, suitable for encoder signals
#define LEFT_ENC_GPIO_B GPIO_NUM_36 // GPIO 36 is input-only, suitable for encoder signals
#define RIGHT_ENC_GPIO_A GPIO_NUM_39 // GPIO 39 is input-only, suitable for encoder signals
#define RIGHT_ENC_GPIO_B GPIO_NUM_25 // GPIO 25 is input-only, suitable for encoder signals

/* test PWM pins */
#define LEFT_MOTOR_PWM GPIO_NUM_13 // GPIO 13 is also connected to the ESP LED, will be used to test PWM output
#define LEFT_MOTOR_DIR GPIO_NUM_12 // GPIO 12 is used to control the direction of the left motor
#define RIGHT_MOTOR_PWM GPIO_NUM_27 // GPIO 27 is used to control the PWM output of the right motor
#define RIGHT_MOTOR_DIR GPIO_NUM_33 // GPIO 33 is used to control the direction of the right motor

#define PWM_FREQ_HZ 50 // 50 Hz for typical RC servo control, adjust as needed for your motors
#define PWM_RESOLUTION LEDC_TIMER_16_BIT // 16-bit resolution for finer control
#define PWM_PERIOD_US 20000 // 20 ms period for 50 Hz
#define PWM_MAX_DUTY ((1 << 16) - 1) // 65535 for 16-bit

#define PULSE_NEUTRAL_US 1500 // 1.5 ms pulse for neutral (stop) position
#define PULSE_MIN_US 1050 // 1.05 ms pulse for minimum speed
#define PULSE_MAX_US 1950 // 1.95 ms pulse for maximum speed

static volatile int32_t left_ticks = 0; // counter for left encoder ticks, updated in ISR
static volatile int32_t right_ticks = 0; // counter for right encoder ticks, updated in ISR

static void IRAM_ATTR left_encoder_isr(void *arg)
{
    int b_state = gpio_get_level(LEFT_ENC_GPIO_B); // read state of channel B to determine directiond
    if (b_state == 0) // if B is low, we are moving in one direction
    {
        left_ticks++; // increment tick count
    }
    else // if B is high, we are moving in the opposite direction
    {
        left_ticks--; // decrement tick count
    }
}

static void IRAM_ATTR right_encoder_isr(void *arg)
{
    int b_state = gpio_get_level(RIGHT_ENC_GPIO_B); // read state of channel B to determine direction
    if (b_state == 0) // if B is low, we are moving in one direction
    {
        right_ticks++; // increment tick count
    }
    else // if B is high, we are moving in the opposite direction
    {
        right_ticks--; // decrement tick count
    }
}

static uint32_t us_to_duty(uint32_t pulse_us)
{
    return (pulse_us * PWM_MAX_DUTY) / PWM_PERIOD_US; // convert pulse width to duty cycle
}

static void motor_pwm_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);

    // Left motor PWM channel
    ledc_channel_config_t left_pwm_conf = {
        .gpio_num = LEFT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&left_pwm_conf);

    // Right motor PWM channel
    ledc_channel_config_t right_pwm_conf = {
        .gpio_num = RIGHT_MOTOR_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&right_pwm_conf);

    // Direction pins
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << LEFT_MOTOR_DIR) | (1ULL << RIGHT_MOTOR_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&dir_conf);
}

static void set_left_motor(int speed_percent)
{
    // speed_percent: -100 to +100
    if (speed_percent > 100)
    {
        speed_percent = 100; // clamp to max
    }
    if (speed_percent < -100)
    {
        speed_percent = -100; // clamp to min
    }

    uint32_t pulse_us = PULSE_NEUTRAL_US + (speed_percent * 450) / 100; // calculate pulse width based on speed percentage

    uint32_t duty = us_to_duty(pulse_us); // convert pulse width to duty cycle
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty); // set duty cycle for left motor
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // update duty cycle to apply changes
}

static void set_right_motor(int speed_percent)
{
    if (speed_percent > 100) 
    {
        speed_percent = 100; // clamp to max
    }
    if (speed_percent < -100)
    {
        speed_percent = -100; // clamp to min
    }

    uint32_t pulse_us = PULSE_NEUTRAL_US + (speed_percent * 450) / 100; // calculate pulse width based on speed percentage  
    uint32_t duty = us_to_duty(pulse_us); // convert pulse width to duty cycle
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty); // set duty cycle for right motor
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1); // update duty cycle to apply changes
}

void app_main(void)
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
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&enc_conf);

    gpio_install_isr_service(0); // install GPIO ISR service with default configuration
    gpio_isr_handler_add(LEFT_ENC_GPIO_A, left_encoder_isr, NULL); // attach ISR handler for left encoder channel A
    gpio_isr_handler_add(RIGHT_ENC_GPIO_A, right_encoder_isr, NULL); // attach ISR handler for right encoder channel A

    motor_pwm_init(); // initialize PWM for motor control

    // Example: both motors forward at 50%
    set_left_motor(0); // set left motor to neutral (stop)
    set_right_motor(0); // set right motor to neutral (stop)
    vTaskDelay(pdMS_TO_TICKS(2000)); // wait for 2 seconds

    while (1)
    {
        // printf("Left ticks: %" PRId32 " | Right ticks: %" PRId32 "\n",
        //        left_ticks, right_ticks);

        // vTaskDelay(pdMS_TO_TICKS(250));

        // PWM LED TEST
        // Left PWM (GPIO 13) is also set to the ESP LED, will be used to test PWM output
        for (int i = 0; i < 100; i++)
        {
            set_left_motor(-i);
            set_right_motor(i);
            printf("Speed: %d%% | L: %" PRId32 " | R: %" PRId32 "\n", 
                   i, left_ticks, right_ticks);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        for (int i = 100; i >= 0; i--)
        {
            set_left_motor(-i);
            set_right_motor(i);
            printf("Speed: %d%% | L: %" PRId32 " | R: %" PRId32 "\n", 
                   i, left_ticks, right_ticks);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}