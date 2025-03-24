#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_partition.h"
#include "esp_flash.h"
#include "esp_timer.h"
// #include "nvs_flash.h"
// #include "driver/spi_common.h"

#include "types.h"
#include "led.h"
#include "uart_console.h"
#include "bldc_pwm.h"
#include "as5600_lib.h"


// -------------------------------------------------------------------------- 
// ----------------------------- DEFINITIONS --------------------------------
// --------------------------------------------------------------------------

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 2           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

#define MOTOR_MCPWM_TIMER_RESOLUTION_HZ 100*1000 // 1MHz, 1 tick = 1us
#define MOTOR_MCPWM_FREQ_HZ             50    // 50Hz PWM
#define MOTOR_MCPWM_DUTY_TICK_MAX       (MOTOR_MCPWM_TIMER_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define MOTOR_MCPWM_GPIO                3
#define MOTOR_REVERSE_GPIO              8

#define LED_TIME_US     2*1000*1000
#define LED_LSB_GPIO    15

#define UART_NUM        0

// --------------------------------------------------------------------------
// ----------------------------- GLOBAL VARIABLES ---------------------------
// --------------------------------------------------------------------------

static const char* TAG_UART_TASK = "uart_task";
static const char* TAG_ADC_TASK = "adc_task";
static const char* TAG_CMD = "cmd";

static TaskHandle_t task_handle_adc;

volatile flags_t gFlag;
led_rgb_t gLed;
uart_console_t gUc;
bldc_pwm_motor_t gMotor;
AS5600_t gAs5600;
system_t gSys;

// --------------------------------------------------------------------------
// ----------------------------- PROTOTYPES ---------------------------------
// --------------------------------------------------------------------------

/**
 * @brief Initialize some variables to start the system
 * 
 */
void init_system(void);

/**
 * @brief Callback for the system one-shot timer
 * 
 * @param arg 
 */
void sys_timer_cb(void *arg);

/**
 * @brief Proccess the command received from the UART console
 * 
 * @param cmd 
 */
void process_cmd(const char *cmd);

/**
 * @brief Task to handle the ADC continuous conversion
 * 
 * @param pvParameters 
 */
void adc_continuous_task(void *pvParameters);

/**
 * @brief Task to handle the UART events
 * 
 * @param pvParameters 
 */
void uart_event_task(void *pvParameters);

// --------------------------------------------------------------------------
// --------------------------------- MAIN -----------------------------------
// --------------------------------------------------------------------------

void app_main(void)
{
    ///< ---------------------- LED ----------------------
    led_init(&gLed, LED_LSB_GPIO, LED_TIME_US, true);
    led_set_blink(&gLed, true, 3);
    led_setup_green(&gLed, LED_TIME_US);

    ///< ---------------------- UART ---------------------
    uconsole_init(&gUc, UART_NUM);
    xTaskCreate(uart_event_task, "uart_event_task", 3*1024, NULL, 1, NULL);

    ///< ---------------------- BLDC ---------------------
    bldc_init(&gMotor, MOTOR_MCPWM_GPIO, MOTOR_REVERSE_GPIO, MOTOR_MCPWM_FREQ_HZ, 0, MOTOR_MCPWM_TIMER_RESOLUTION_HZ);
    bldc_enable(&gMotor);
    bldc_set_duty(&gMotor, 1); // Set duty to 0.1%, so the motor will not move

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_OFF, ///< Hysteresis off
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_16X, ///< Slow filter 16x
        .FTH = AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY, ///< Slow filter only
        .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    };
    AS5600_SetConf(&gAs5600, conf);

    ///< ---------------------- SYSTEM -------------------
    // 'System' refers to more general variables and functions that are used to control the system, which
    // consists of the BLDC motor, the AS5600 sensor, the LED, and the UART console.
    init_system();

}

// --------------------------------------------------------------------------
// ------------------------------- FUNCTIONS --------------------------------
// --------------------------------------------------------------------------

void init_system(void)
{
    gSys.STATE = NONE; ///< Initialize the state machine
    gSys.current_bytes_written = 0; ///< Initialize the number of samples readed from the ADC

    // Get the partition table and erase the partition to store new data
    gSys.part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "angle_pos");
    ESP_ERROR_CHECK(esp_flash_erase_region(gSys.part->flash_chip, gSys.part->address, gSys.part->size));
    char part_label[] = "Time(us)\tAngle(deg)\tDuty\n";
    esp_err_t rest = esp_partition_write(gSys.part, 0, part_label, strlen(part_label));
    gSys.current_bytes_written += strlen(part_label);

    // Print the result of the operation
    if (rest == ESP_OK) {
        ESP_LOGI("init_system", "Text written successfully");
    }
    else {
        ESP_LOGI("init_system", "Error writing text");
    }

    // Create a one-shot timer to control the sequence
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sys_timer_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "sys-one-shot" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
    gSys.oneshot_timer = oneshot_timer;

    ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer, NONE_TO_STEPS_US));
}

void sys_timer_cb(void *arg)
{
    switch(gSys.STATE)
    {
        case NONE:
            ESP_LOGI("sys_timer_cb", "INIT_PWM_BLDC_STEP_1");
            bldc_set_duty(&gMotor, 60); ///< Set the duty cycle to 6%
            gSys.STATE = INIT_PWM_BLDC_STEP_1;
            ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer, STEP1_TO_STEP2_US));
            break;

        case INIT_PWM_BLDC_STEP_1: // Step 1: Start the timer to go to step 2 and set duty >5.7%
            ESP_LOGI("sys_timer_cb", "INIT_PWM_BLDC_STEP_2");
            bldc_set_duty(&gMotor, 50); ///< Set the duty cycle to 5%
            gSys.STATE = INIT_PWM_BLDC_STEP_2;
            ESP_ERROR_CHECK(esp_timer_start_once(gSys.oneshot_timer, STEPS_TO_SEQ_US));
            break;

        case INIT_PWM_BLDC_STEP_2: 
            ESP_LOGI("sys_timer_cb", "SEQ_BLDC_1");
            ESP_ERROR_CHECK(esp_timer_delete(gSys.oneshot_timer));
            gSys.STATE = SEQ_BLDC_1;
            bldc_set_duty(&gMotor, 65); ///< Set the duty cycle to 6.5%
            gSys.start_adc_time = esp_rtc_get_time_us();
            break;

        default:
            ESP_LOGI("sys_timer_cb", "default");
            break;
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
                process_cmd(cmd);
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

void process_cmd(const char *cmd)
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
            bldc_set_duty(&gMotor, value);
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
        AS5600_reg_t addr = AS5600_RegStrToAddr(&gAs5600, reg); ///< Map the str to int address
        if (addr == -1) {
            ESP_LOGI(TAG_CMD, "Invalid register");
            return;
        }

        ///< Read or write the register
        uint16_t data;
        if (rw == 'r') {
            ESP_LOGI(TAG_CMD, "addr-> %02x", (uint8_t)addr);
            AS5600_ReadReg(&gAs5600, addr, &data);
            ESP_LOGI(TAG_CMD, "readed-> %04x", data);
        }
        ///< For write commands, it is necessary to get the value to write
        else if (rw == 'w') {
            ///< Check the minimum length of the command
            if (len_uc_data != 15) { ///< 15 for a command like "as5 w [regi] [0000]"
                ESP_LOGI(TAG_CMD, "Invalid AS5600 cmd");
                return;
            }
            ///< Get the exadecimal value from 3 characters
            uint8_t len_value = 4;
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
            AS5600_WriteReg(&gAs5600, addr, value);
            ESP_LOGI(TAG_CMD, "addr-> %02x, value-> %04x", (uint8_t)addr, value);
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
