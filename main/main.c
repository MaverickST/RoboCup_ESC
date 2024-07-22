#include <stdio.h>
#include "sdkconfig.h"  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"

#include "types.h"
#include "led.h"
#include "uart_console.h"
#include "bldc_pwm.h"

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 100*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                3

#define LED_TIME_US     2*1000*1000
#define LED_LSB_GPIO    47

#define UART_NUM        0

static const char* TAG_UART_TASK = "uart_task";

volatile flags_t gFlag;
led_rgb_t gLed;
uart_console_t gUc;
bldc_pwm_motor_t gMotor;

void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    while(true) {
        if(xQueueReceive(gUc.uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
            case UART_DATA: ///< Event of UART receving data
                ESP_LOGI(TAG_UART_TASK, "UART_DATA");
                uconsole_read_data(&gUc);
                uint16_t value = atoi((const char *)gUc.data);
                ESP_LOGI(TAG_UART_TASK, "value: %d", value);
                if (value != 0) { ///< If value=0, that means data is not a number
                    bldc_set_speed(&gMotor, value);
                }
                break;

            case UART_FIFO_OVF: ///< Event of HW FIFO overflow
                ESP_LOGI(TAG_UART_TASK, "UART_FIFO_OVF");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            case UART_BUFFER_FULL: ///< Event of UART ring buffer full
                ESP_LOGI(TAG_UART_TASK, "UART_BUFFER_FULL");
                uart_flush_input(gUc.uart_num);
                xQueueReset(gUc.uart_queue);
                break;

            default:
                ESP_LOGI(TAG_UART_TASK, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ///< LED Initialization
    led_init(&gLed, LED_LSB_GPIO, LED_TIME_US, false);
    // led_set_blink(&gLed, true, 5);
    // led_setup_red(&gLed, LED_TIME_US);
    led_on(&gLed);

    ///< UART console initialization
    uconsole_init(&gUc, UART_NUM);
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    ///< BLDC motor initialization
    bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, MOTOR_MCPWM_TIMER_RESOLUTION_HZ);
    bldc_enable(&gMotor);
    bldc_set_speed(&gMotor, 200);

}