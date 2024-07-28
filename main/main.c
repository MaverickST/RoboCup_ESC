#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#include "as5600.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 100*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                3

#define LED_TIME_US     2*1000*1000
#define LED_LSB_GPIO    15

#define UART_NUM        0

static const char* TAG_UART_TASK = "uart_task";
static const char* TAG_CMD = "cmd";

volatile flags_t gFlag;
led_rgb_t gLed;
uart_console_t gUc;
bldc_pwm_motor_t gMotor;
as5600_t gAs5600;


void proccess_cmd(const char *cmd)
{
    ///< Check if the data is valid
    uint8_t len_uc_data = strlen((const char *)gUc.data);
    if (len_uc_data < 5) { ///< If the data is less than 5 characters, that means the data is not valid
        ESP_LOGI(TAG_CMD, "Invalid CMD");
        return;
    }
    ///< Command to set the speed of the motor
    if (strcmp(cmd, "pwm") == 0) {
        char str_value[len_uc_data - 4]; ///< 4 is the length of the command "pwm "
        strncpy(str_value, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the value after the command

        uint16_t value = atoi(str_value);
        ESP_LOGI(TAG_CMD, "value-> %d", value);
        if (value != 0) { ///< If value=0, that means data is not a number
            bldc_set_speed(&gMotor, value);
        }
    }
    ///< Command to read the angle from the AS5600 sensor
    else if(strcmp(cmd, "as5") == 0){
        ///< Check the minimum length of the command
        if (len_uc_data <= 8) { ///< 4 for the command "as5 " and 4 for the register
            ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
            return;
        }
        ///< From cmd, get if it is read or write: "as5 r" or "as5 w"
        char rw = *(gUc.data + 4);

        ///< Then get the register to read or write: "as5 r zmco", "as5 w zpos", ....
        uint8_t len_reg = 4;
        char reg[len_reg]; ///< 4 is the length of the register, +1 for the null terminator
        strncpy(reg, (const char *)gUc.data + strlen("as5 r "), len_reg); ///< Get the register after the command
        reg[len_reg] = '\0'; ///< Add the null terminator
        as5600_reg_t addr = as5600_reg_str_to_addr(&gAs5600, reg); ///< Map the str to int address
        if (addr == -1) {
            ESP_LOGI(TAG_CMD, "Invalid register");
            return;
        }

        ///< Read or write the register
        uint16_t data;
        if (rw == 'r') {
            ESP_LOGI(TAG_CMD, "addr-> %02x", (uint8_t)addr);
            as5600_read_reg(&gAs5600, addr, &data);
            ESP_LOGI(TAG_CMD, "readed-> %03x", data);
        }
        ///< For write commands, it is necessary to get the value to write
        else if (rw == 'w') {
            ///< Check the minimum length of the command
            if (len_uc_data != 14) { ///< 4 for the command "as5 w [regi] [000]" and 2 for the register and 5 for the value
                ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
                return;
            }
            ///< Get the exadecimal value from 3 characters
            uint8_t len_value = 3;
            char str_value[len_value]; ///< value to write in chars
            strncpy(str_value, (const char *)gUc.data + strlen("as5 w regi "), len_value); ///< Get the value after the command
            str_value[len_value] = '\0'; ///< Add the null terminator
            uint16_t value = 0; ///< value to write in int
            for (int i = 0; i < len_value; i++) {
                if (str_value[i] >= '0' && str_value[i] <= '9') {
                    value += (str_value[i] - '0') << (4 * (len_value - i - 1));
                }
                else if (str_value[i] >= 'a' && str_value[i] <= 'f') {
                    value += (str_value[i] - 'a' + 10) << (4 * (len_value - i - 1));
                }
                else {
                    ESP_LOGI(TAG_CMD, "Invalid value");
                    return;
                }
            }
            as5600_write_reg(&gAs5600, addr, value);
            ESP_LOGI(TAG_CMD, "addr-> %02x, value-> %03x", (uint8_t)addr, value);
        }
        else {
            ESP_LOGI(TAG_CMD, "rw not recognized");
        }

    }
    ///< Command to turn on the LED
    else if(strcmp(cmd, "led") == 0) {
        if (len_uc_data < 7) { ///< 4 for the command "led " and 5 for the color
            ESP_LOGI(TAG_CMD, "Invalid LED cmd");
            return;
        }
        char str_color[len_uc_data - 4]; ///< 4 is the length of the command "led "
        strncpy(str_color, (const char *)gUc.data + 4, len_uc_data - 4); ///< Get the color after the command
        str_color[len_uc_data - 4] = '\0'; ///< Add the null terminator
        if (strcmp(str_color, "red") == 0) {
            led_setup_red(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "green") == 0) {
            led_setup_green(&gLed, LED_TIME_US);
        }
        else if (strcmp(str_color, "blue") == 0) {
            led_setup_blue(&gLed, LED_TIME_US);
        }
        else {
            ESP_LOGI(TAG_CMD, "color not recognized");
        }
    }
    else {
        ESP_LOGI(TAG_CMD, "cmd not recognized");
    }
}


void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    while(true) {
        if(xQueueReceive(gUc.uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch(event.type) {
            case UART_DATA: ///< Event of UART receving data
                ESP_LOGI(TAG_UART_TASK, "UART_DATA");
                uconsole_read_data(&gUc);

                char cmd[4];
                strncpy(cmd, (const char *)gUc.data, 3); ///< Get the first 3 characters
                cmd[3] = '\0'; ///< Add the null terminator
                ESP_LOGI(TAG_UART_TASK, "cmd-> %s", cmd);
                proccess_cmd(cmd);
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
    led_init(&gLed, LED_LSB_GPIO, LED_TIME_US, true);
    led_set_blink(&gLed, true, 3);
    led_setup_green(&gLed, LED_TIME_US);

    ///< UART console initialization
    uconsole_init(&gUc, UART_NUM);
    xTaskCreate(uart_event_task, "uart_event_task", 3*1024, NULL, 12, NULL);

    ///< BLDC motor initialization
    bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, MOTOR_MCPWM_TIMER_RESOLUTION_HZ);
    bldc_enable(&gMotor);
    bldc_set_speed(&gMotor, 1);

    ///< AS5600 sensor initialization
    as5600_init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO);

}