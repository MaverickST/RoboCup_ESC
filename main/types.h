/**
 * \file        types.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "esp_partition.h"

typedef union
{
    uint8_t B;
    struct
    {
        uint8_t uc_data : 1; ///< Data received from the UART console
        uint8_t         : 7; ///< Reserved    
    };
} flags_t;

extern volatile flags_t gFlag;


typedef struct
{
    enum 
    {
        NONE, ///< Nothing to do
        // To initialize the motor, it is necessary to send two PWM signals to the BLDC motor (ESC)
        INIT_PWM_BLDC_STEP_1, ///< The first PWM value must: >5.7% duty cycle
        INIT_PWM_BLDC_STEP_2, ///< After 1s, send the second PWM value must: <5.7% duty cycle.

        // Then, the motor will start to rotate on a predefined way, following the sequence below.
        SEQ_BLDC_1, ///< Sequence 1: PWM1 = 6.5%
        SEQ_BLDC_2, ///< Sequence 2: PWM2 = 7.5%
        SEQ_BLDC_3, ///< Sequence 3: PWM3 = 8.5%
        SEQ_BLDC_LAST, ///< The last step of the sequence

        BLDC_STOP,  ///< Stop the motor
    } STATE;

    enum
    {
        NONE_TO_STEPS_US   = 1*100*1000, ///< The time between the initial state and the first step is 100ms
        STEP1_TO_STEP2_US  = 3*1000*1000, ///< The time between step 1 and step 2 is 1s
        STEPS_TO_SEQ_US    = 5*1000*1000, ///< The time between the first two steps and the sequence is 5s
    } TIME;

    uint16_t duty_to_save; ///< PWM value to save in the NVS
    uint32_t current_bytes_written; ///< Number of samples readed from the ADC
    esp_timer_handle_t oneshot_timer;    ///< Timer to control the sequence
    const esp_partition_t *part;   ///< Pointer to the partition table

    uint32_t start_adc_time;
    uint32_t done_adc_time;
}system_t;

/**
 * @brief Flash
 * Some useful commands to read and write data to the flash memory via python console:
 *      - Erase partition with name 'storage'
 *              parttool.py --port "/dev/ttyUSB1" erase_partition --partition-name=storage
 *      - Read partition with type 'data' and subtype 'spiffs' and save to file 'spiffs.bin' 
 *              parttool.py --port "/dev/ttyUSB1" read_partition --partition-type=data --partition-subtype=spiffs --output "spiffs.bin"
 *      - Write to partition 'factory' the contents of a file named 'factory.bin'
 *              parttool.py --port "/dev/ttyUSB1" write_partition --partition-name=factory --input "factory.bin"
 *      - Print the size of default boot partition
 *              parttool.py --port "/dev/ttyUSB1" get_partition_info --partition-boot-default --info size
 *      - Print the size of the partition with name 'storage'
 *              parttool.py --port "/dev/ttyUSB1" get_partition_info --partition-name=storage --info size 
 * 
 * You just have to replace the port with the one you are using: "/dev/ttyUSB1" --> com5, com6, etc.
 * 
 * You may need to specify the path to the python script:
 *              python C:\Espressif\esp-idf-v5.2.2\components\partition_table\parttool.py --port com5.... 
 * 
 * python C:\Espressif\esp-idf-v5.2.2\components\partition_table\parttool.py --port com5 read_partition --partition-name=angle_pos --partition-subtype=nvs --output "angle.txt"
 * 
 * 
 */


/** 
esp_err_t ret = adc_continuous_read(gAs5600.adc_cont_handle, gAs5600.buffer, AS5600_ADC_READ_SIZE_BYTES, &gAs5600.ret_num, portMAX_DELAY);
uint32_t ret_num = gAs5600.ret_num;
if (ret == ESP_OK) {

    ESP_LOGI(TAG_ADC_TASK, "ret is %x, ret_num is %d bytes, in time %d", ret, (int)ret_num, (int)(gSys.done_adc_time - gSys.start_adc_time));
    gSys.start_adc_time = esp_rtc_get_time_us();

    """"
        -  'data_frame' is a buffer to save the data readed from the ADC. The data is saved in a .txt file .
        -  See 'types.h' file for more details about python commands.
        -  20 is a right-minded of the number of characters (bytes) needed per data (per line in the .txt file), but
        in many cases, it will be less than 20.
    """"
    char data_frame[(ret_num / SOC_ADC_DIGI_DATA_BYTES_PER_CONV)*20];
    uint32_t cnt_bytes = 0;
    static uint32_t time = 0;

    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&gAs5600.buffer[i];
        uint32_t chan_num = p->type2.channel;
        uint32_t data = p->type2.data;
        ///< Check the channel number validation, the data is invalid if the channel num exceed the maximum channel 
        if (!(chan_num < SOC_ADC_CHANNEL_NUM(AS5600_ADC_CONF_UNIT))) {
            ESP_LOGW(TAG_ADC_TASK, "Invalid data [%d_%"PRIu32"_%"PRIx32"]", gAs5600.unit, chan_num, data);
        }
        // Save the data in the data_frame array taking into account the time, angle and duty.
        time += AS5600_ADC_SAMPLE_PERIOD_US;
        uint16_t duty = gSys.duty_to_save;
        uint16_t angle = 0;
        as5600_adc_raw_to_angle(&gAs5600, data, &angle);

        uint8_t length = snprintf(NULL, 0, "%d\t\t%d\t\t%d\n", (int)time, (int)angle, (int)duty);
        char str[length + 1];
        snprintf(str, length + 1, "%d\t\t%d\t\t%d\n", (int)time, (int)angle, (int)duty);
        for(int j = 0; j < length; j++) { // copy the string to the data_frame array
            data_frame[cnt_bytes++] = str[j];
        }
    }
    // Write the data to the flash memory
    esp_flash_write(gSys.part->flash_chip, data_frame, gSys.part->address + gSys.current_bytes_written, cnt_bytes);
    gSys.current_bytes_written += cnt_bytes;
    ESP_LOGI(TAG_ADC_TASK, "current_bytes_written: %d", (int)gSys.current_bytes_written);
}

*/

#endif // __TYPES_H__