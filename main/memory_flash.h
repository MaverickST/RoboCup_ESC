/**
 * \file        memory_flash.h
 * \brief       Used to read and write data to the flash memory
 * \details     
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
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */


#ifndef __MEMORY_FLASH_H__
#define __MEMORY_FLASH_H__

#include <stdint.h>
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_partition.h"


#endif // __MEMORY_FLASH_H__