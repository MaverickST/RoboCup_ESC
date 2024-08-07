/**
 * \file        as5600.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"
// #include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "as5600_types.h"

static const char* TAG_AS5600 = "AS5600";

#define I2C_MASTER_FREQ_HZ  400*1000    /*!< I2C master clock frequency */
#define I2C_TIMEOUT_MS      100         /*!< I2C timeout in milliseconds */

#define AS5600_SENSOR_ADDR  0x36        /*!< slave address for AS5600 sensor */

#define AS5600_ADC_SAMPLE_FREQ_HZ      1000        /*!< ADC sample frequency in Hz */
#define AS5600_ADC_CONF_UNIT           ADC_UNIT_1  /*!< ADC unit for ADC1 */
#define AS5600_ADC_RESOLUTION_12_BIT   4095        /*!< 12-bit resolution for ADC */  
#define AS5600_ADC_READ_SIZE_BYTES     128*SOC_ADC_DIGI_DATA_BYTES_PER_CONV   /*!< Read size in bytes */
#define AS5600_ADC_MAX_BUF_SIZE        4*AS5600_ADC_READ_SIZE_BYTES                   /*!< Maximum buffer size for ADC */

#define AS5600_ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define AS5600_ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define AS5600_ADC_ATTEN               ADC_ATTEN_DB_0
#define AS5600_ADC_BIT_WIDTH           CONFIG_SOC_ADC_DIGI_MIN_BITWIDTH
#define AS5600_ADC_CHANNEL_COUNT       1

typedef struct
{
    i2c_port_t i2c_num;
    uint8_t scl;
    uint8_t sda;
    uint8_t out;
    uint8_t buffer[AS5600_ADC_READ_SIZE_BYTES];
    uint32_t ret_num;
    adc_channel_t chan;
    adc_unit_t unit;
    adc_oneshot_unit_handle_t adc_handle;
    adc_continuous_handle_t adc_cont_handle;

    i2c_master_dev_handle_t dev_handle;
    as5600_reg_t reg;

} as5600_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c_num I2C port number
 */
void as5600_init(as5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out);

/**
 * @brief Deinitialize the I2C master driver
 * 
 */
void as5600_deinit(as5600_t *as5600);

/**
 * @brief Get the output value from the AS5600 sensor by reading the ADC
 * 
 * @param out_value Output value
 */
void as5600_get_out_value(as5600_t *as5600, uint16_t *out_value);

/**
 * @brief Start the ADC conversion in continuous mode.
 * 
 * @param as5600 
 */
static inline void as5600_adc_continuous_start(as5600_t *as5600)
{
    adc_continuous_start(as5600->adc_cont_handle);
}

/**
 * @brief Convert register string to register address
 * 
 * @param reg_str Register string
 * @return as5600_reg_t Register address
 */
as5600_reg_t as5600_reg_str_to_addr(as5600_t *as5600, const char *reg_str);

// -------------------------------------------------------------
// ---------------------- I2C FUNCTIONS ------------------------
// -------------------------------------------------------------

/**
 * @brief Read register
 * 
 * @param reg Register address
 * @param data Pointer to the data
 */
void as5600_read_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t *data);

/**
 * @brief Write register
 * 
 * @param reg Register address
 * @param data Data to write
 */
void as5600_write_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t data);

/**
 * @brief Check if the register is valid for reading
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool as5600_is_valid_read_reg(as5600_t *as5600, as5600_reg_t reg);

/**
 * @brief Check if the register is valid for writing
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool as5600_is_valid_write_reg(as5600_t *as5600, as5600_reg_t reg);

/**
 * @brief Transmit and receive data.
 * Send the register address to read from and receive the data.
 * _______________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | write 1 bytes (reg) + ack | read size bytes + nack | stop |
 * --------|--------------------------|---------------------------|------------------------|------|
 * 
 * @param i2c_num 
 * @param reg 
 * @param data_rd 
 * @param size 
 * @return esp_err_t 
 */
esp_err_t i2c_master_write_read(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_rd, uint8_t size);

// -------------------------------------------------------------
// ---------------------- CONFIG REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Set the start position by writing the ZPOS register
 * 
 * @param start_position 
 */
void as5600_set_start_position(as5600_t *as5600, uint16_t start_position);

/**
 * @brief Get the start position by reading the ZPOS register
 * 
 * @param start_position 
 */
void as5600_get_start_position(as5600_t *as5600, uint16_t *start_position);

/**
 * @brief Set the stop position by writing the MPOS register
 * 
 * @param stop_position 
 */
void as5600_set_stop_position(as5600_t *as5600, uint16_t stop_position);

/**
 * @brief Get the stop position by reading the MPOS register
 * 
 * @param stop_position 
 */
void as5600_get_stop_position(as5600_t *as5600, uint16_t *stop_position);

/**
 * @brief Set the maximum angle by writing the MANG register
 * 
 * @param max_angle 
 */
void as5600_set_max_angle(as5600_t *as5600, uint16_t max_angle);

/**
 * @brief Get the maximum angle by reading the MANG register
 * 
 * @param max_angle 
 */
void as5600_get_max_angle(as5600_t *as5600, uint16_t *max_angle);

/**
 * @brief Set the configuration by writing the CONF register
 * 
 * @param conf Configuration
 */
void as5600_set_conf(as5600_t *as5600, as5600_config_t conf);

/**
 * @brief Get the configuration by reading the CONF register
 * 
 * @param conf Configuration
 */
void as5600_get_conf(as5600_t *as5600, as5600_config_t *conf);


// -------------------------------------------------------------
// ---------------------- OUTPUT REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read RAW ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_raw_angle(as5600_t *as5600, uint16_t *raw_angle);

/**
 * @brief Read ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_angle(as5600_t *as5600, uint16_t *angle);


// -------------------------------------------------------------
// ---------------------- STATUS REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read STATUS register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_status(as5600_t *as5600, uint8_t *status);

/**
 * @brief Read AGC register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_agc(as5600_t *as5600, uint8_t *agc);

/**
 * @brief Read MAGNITUDE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void as5600_get_magnitude(as5600_t *as5600, uint16_t *magnitude);


#endif // __AS5600_H__
