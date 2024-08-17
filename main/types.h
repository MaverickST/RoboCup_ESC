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
    uint32_t actual_num_samples; ///< Number of samples readed from the ADC
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


#endif // __TYPES_H__