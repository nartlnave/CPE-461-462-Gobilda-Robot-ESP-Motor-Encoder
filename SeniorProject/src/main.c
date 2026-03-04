#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

static const char *TAG = "ENC_TEST";

// Pick two input GPIOs for encoder A/B.
// Using input-only pins is fine (34/35). Change to match your wiring.
#define ENCODER_A_GPIO GPIO_NUM_34
#define ENCODER_B_GPIO GPIO_NUM_35

#define PCNT_UNIT      PCNT_UNIT_0
#define PCNT_CHANNEL   PCNT_CHANNEL_0

static void pcnt_setup(void)
{
    // Configure PCNT for quadrature-ish counting:
    // - pulse_gpio_num = A
    // - ctrl_gpio_num  = B (direction control)
    // - count on A rising edges
    // - direction depends on B level
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A_GPIO,
        .ctrl_gpio_num  = ENCODER_B_GPIO,
        .channel        = PCNT_CHANNEL,
        .unit           = PCNT_UNIT,

        // Count on rising edges only (less sensitive to noise than both edges)
        .pos_mode       = PCNT_COUNT_INC,
        .neg_mode       = PCNT_COUNT_DIS,

        // If ctrl (B) is LOW: keep direction
        .lctrl_mode     = PCNT_MODE_KEEP,
        // If ctrl (B) is HIGH: reverse direction
        .hctrl_mode     = PCNT_MODE_REVERSE,

        // Keep within 16-bit range
        .counter_h_lim  = 32767,
        .counter_l_lim  = -32768,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // Optional: enable internal pullups if your encoder outputs are open-drain / need biasing.
    // If your encoder actively drives HIGH/LOW, you can disable these.
    gpio_set_pull_mode(ENCODER_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_B_GPIO, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT));

    ESP_LOGI(TAG, "PCNT configured on A=%d, B=%d", ENCODER_A_GPIO, ENCODER_B_GPIO);
}

static void encoder_print_task(void *arg)
{
    int16_t count = 0;
    while (1) {
        pcnt_get_counter_value(PCNT_UNIT, &count);
        printf("ticks: %d\n", count);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void serial_command_task(void *arg)
{
    // Simple UART stdin init so you can type 'r' to reset count.
    // PlatformIO monitor sends to UART0 by default.
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    while (1) {
        int c = getchar();
        if (c == 'r' || c == 'R') {
            pcnt_counter_clear(PCNT_UNIT);
            printf("count reset\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    pcnt_setup();

    xTaskCreate(encoder_print_task, "enc_print", 4096, NULL, 5, NULL);
    xTaskCreate(serial_command_task, "cmd", 4096, NULL, 4, NULL);
}