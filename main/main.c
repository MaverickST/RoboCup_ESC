#include <stdio.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "gpio_led.h"

#define LED_LSB_GPIO    16
static const char* TAG_test = "test: ";

led_rgb_t gLed;

void app_main(void)
{
    led_init(&gLed, LED_LSB_GPIO, 500000);
    led_set_blink(&gLed, true, 5);
    led_setup_red(&gLed, 500000);
    while (1)
    {
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(2000000));
        esp_light_sleep_start();
        ESP_LOGI(TAG_test, "Wake up");
    }

}