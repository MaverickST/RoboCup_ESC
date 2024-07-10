#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "types.h"
#include "led.h"
#include "uart_console.h"
// #include "bldc_pwm.h"

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define MOTOR_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                32

#define LED_TIME_US     500000
#define LED_LSB_GPIO    25

void uc_handler(void *arg)
{
    uart_console_t *uc = (uart_console_t *)arg;

    uconsole_read_data(uc);
    ESP_LOGI("UART", "Data received: %s", uc->data);
}

static const char* TAG_test = "test: ";

volatile flags_t gFlag;
led_rgb_t gLed;
uart_console_t gUc;
// bldc_pwm_motor_t gMotor;

void app_main(void)
{
    ///< LED Initialization
    led_init(&gLed, LED_LSB_GPIO, LED_TIME_US);
    led_set_blink(&gLed, true, 5);
    led_setup_red(&gLed, LED_TIME_US);

    ///< UART console initialization
    // uconsole_init(&gUc);

    ///< BLDC motor initialization
    // bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, MOTOR_MCPWM_TIMER_RESOLUTION_HZ);
    // bldc_enable(&gMotor);
    // bldc_set_speed(&gMotor, 100);

    while (1)
    {
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(LED_TIME_US*10));
        esp_light_sleep_start();
        // vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG_test, "Wake up");

        while(gFlag.B) {
            
            if (gFlag.uc_data) {
                gFlag.uc_data = false;
                ESP_LOGI(TAG_test, "Data received from the UART console");
            }
        }
    }

}