#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define LEFT_ENC_GPIO   GPIO_NUM_34
#define RIGHT_ENC_GPIO  GPIO_NUM_39

static volatile int32_t left_ticks = 0;
static volatile int32_t right_ticks = 0;

static void IRAM_ATTR left_encoder_isr(void *arg) {
    left_ticks++;
}

static void IRAM_ATTR right_encoder_isr(void *arg) {
    right_ticks++;
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,   // count rising edges
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LEFT_ENC_GPIO) | (1ULL << RIGHT_ENC_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_ENC_GPIO, left_encoder_isr, NULL);
    gpio_isr_handler_add(RIGHT_ENC_GPIO, right_encoder_isr, NULL);

    while (1) {
        printf("Left ticks: %" PRId32 " | Right ticks: %" PRId32 "\n", left_ticks, right_ticks);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}