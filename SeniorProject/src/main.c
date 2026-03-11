#include <stdio.h>
#include <inttypes.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define LEFT_ENC_GPIO_A   GPIO_NUM_34
#define LEFT_ENC_GPIO_B   GPIO_NUM_36

#define RIGHT_ENC_GPIO_A  GPIO_NUM_39
#define RIGHT_ENC_GPIO_B  GPIO_NUM_25

static volatile int32_t left_ticks = 0;
static volatile int32_t right_ticks = 0;


static void IRAM_ATTR left_encoder_isr(void *arg) {
    int b_state = gpio_get_level(LEFT_ENC_GPIO_B);

    if (b_state == 0) { 
        left_ticks--;   //  note that b_state == 0 decrements
    } else {
        left_ticks++;
    }
}

static void IRAM_ATTR right_encoder_isr(void *arg) {
    int b_state = gpio_get_level(RIGHT_ENC_GPIO_B);

    if (b_state == 0) {
        right_ticks++;  //  note that b_state == 0 increments
    } else {
        right_ticks--;
    }
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,   // interrupt only on A rising edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << LEFT_ENC_GPIO_A)  |
            (1ULL << LEFT_ENC_GPIO_B)  |
            (1ULL << RIGHT_ENC_GPIO_A) |
            (1ULL << RIGHT_ENC_GPIO_B),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_ENC_GPIO_A, left_encoder_isr, NULL);
    gpio_isr_handler_add(RIGHT_ENC_GPIO_A, right_encoder_isr, NULL);

    while (1) {
        printf("Left ticks: %" PRId32 " | Right ticks: %" PRId32 "\n",
               left_ticks, right_ticks);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}