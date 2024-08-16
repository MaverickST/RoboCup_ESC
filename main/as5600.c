#include "as5600.h"

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"


void as5600_init(as5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out)
{
    as5600->i2c_num = i2c_num;
    as5600->scl = scl;
    as5600->sda = sda;
    as5600->out = out;

    // ------------- I2C master configuration ------------- //
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_num,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    static i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    as5600->dev_handle = dev_handle;

    // ------------- ADC pin OUT configuration ------------- //
    // The DIG ADC2 controller of ESP32-S3 doesn’t work properly (pag. 1444).
    // So we need to use the ADC1 controller (GPIO-1 to GPIO-10, pag. 318).

    // From GPIO to ADC channel
    as5600->buffer[0] = 0;
    memset(as5600->buffer, 0xcc, AS5600_ADC_READ_SIZE_BYTES);
    as5600->unit = AS5600_ADC_CONF_UNIT;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(as5600->out, &as5600->unit, &as5600->chan));
    ESP_LOGI(TAG_AS5600, "ADC channel: %d", as5600->chan);

    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = AS5600_ADC_MAX_BUF_SIZE,
        .conv_frame_size = AS5600_ADC_READ_SIZE_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = AS5600_ADC_SAMPLE_FREQ_HZ,
        .conv_mode = AS5600_ADC_CONV_MODE,
        .format = AS5600_ADC_OUTPUT_TYPE,
    };
    dig_cfg.pattern_num = as5600->chan; // AS5600_ADC_CHANNEL_COUNT

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0] = (adc_digi_pattern_config_t) {
        .atten = AS5600_ADC_ATTEN,
        .channel = as5600->chan,
        .unit = AS5600_ADC_CONF_UNIT,
        .bit_width = AS5600_ADC_BIT_WIDTH,
    };
    dig_cfg.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    as5600->adc_cont_handle = handle;

}

void as5600_deinit(as5600_t *as5600)
{
    i2c_del_master_bus(as5600->dev_handle);
    adc_oneshot_del_unit(as5600->adc_handle);
    adc_continuous_deinit(as5600->adc_cont_handle);
}

void as5600_get_out_value(as5600_t *as5600, uint16_t *out_value)
{
    adc_oneshot_read(as5600->adc_handle, as5600->chan, (int *)out_value);
    *out_value = (*out_value * AS5600_ADC_RESOLUTION_12_BIT) / 360;
}

as5600_reg_t as5600_reg_str_to_addr(as5600_t *as5600, const char *reg_str)
{
    if (strcmp(reg_str, "zmco") == 0) {
        as5600->reg = AS5600_REG_ZMCO;
    }
    else if (strcmp(reg_str, "zpos") == 0) {
        as5600->reg = AS5600_REG_ZPOS_H;
    }
    else if (strcmp(reg_str, "mpos") == 0) {
        as5600->reg = AS5600_REG_MPOS_H;
    }
    else if (strcmp(reg_str, "mang") == 0) {
        as5600->reg = AS5600_REG_MANG_H;
    }
    else if (strcmp(reg_str, "conf") == 0) {
        as5600->reg = AS5600_REG_CONF_H;
    }
    else if (strcmp(reg_str, "stat") == 0) {
        as5600->reg = AS5600_REG_STATUS;
    }
    else if (strcmp(reg_str, "rang") == 0) {
        as5600->reg = AS5600_REG_RAW_ANGLE_H;
    }
    else if (strcmp(reg_str, "angl") == 0) {
        as5600->reg = AS5600_REG_ANGLE_H;
    }
    else if (strcmp(reg_str, "agco") == 0) {
        as5600->reg = AS5600_REG_AGC;
    }
    else if (strcmp(reg_str, "magn") == 0) {
        as5600->reg = AS5600_REG_MAGNITUDE_H;
    }
    else if (strcmp(reg_str, "burn") == 0) {
        as5600->reg = AS5600_REG_BURN;
    }
    else {
        return -1;
    }
    return as5600->reg;
}

void as5600_read_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t *data)
{
    if (!as5600_is_valid_read_reg(as5600, reg)) {
        ESP_LOGI(TAG_AS5600, "Invalid register");
        return;
    }
    uint8_t write_buffer[] = {reg};

    ///< Read 1 byte for ZMCO, STATUS, AGC
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_STATUS || reg == AS5600_REG_AGC) {
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
    ///< Read 2 bytes for the rest of the readeable registers
    else {
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        *data = (*data << 8) | (*data >> 8);
    }
}

void as5600_write_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t data)
{
    if (!as5600_is_valid_write_reg(as5600, reg)) {
        ESP_LOGI(TAG_AS5600, "Invalid register");
        return;
    }
    ///< Write 1 byte for BURN
    if (reg == AS5600_REG_BURN) {
        uint8_t write_buffer[] = {reg, data};
        i2c_master_transmit(as5600->dev_handle, write_buffer, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
    ///< Write 2 bytes for the rest of the writeable registers
    else {
        uint8_t write_buffer[] = {reg, data >> 8, data};
        i2c_master_transmit(as5600->dev_handle, write_buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
}

bool as5600_is_valid_read_reg(as5600_t *as5600, as5600_reg_t reg)
{
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L || 
        reg == AS5600_REG_MPOS_H || reg == AS5600_REG_MPOS_L || reg == AS5600_REG_MANG_H || 
        reg == AS5600_REG_MANG_L || reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L || 
        reg == AS5600_REG_STATUS || reg == AS5600_REG_RAW_ANGLE_H || reg == AS5600_REG_RAW_ANGLE_L || 
        reg == AS5600_REG_ANGLE_H || reg == AS5600_REG_ANGLE_L || reg == AS5600_REG_AGC || 
        reg == AS5600_REG_MAGNITUDE_H || reg == AS5600_REG_MAGNITUDE_L)
    {
        return true;
    }
    return false;
}

bool as5600_is_valid_write_reg(as5600_t *as5600, as5600_reg_t reg)
{
    if (reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L || reg == AS5600_REG_MPOS_H || 
        reg == AS5600_REG_MPOS_L || reg == AS5600_REG_MANG_H || reg == AS5600_REG_MANG_L || 
        reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L || reg == AS5600_REG_BURN)
    {
        return true;
    }
    return false;
}

// -------------------------------------------------------------
// ---------------------- CONFIG REGISTERS ---------------------
// -------------------------------------------------------------

void as5600_set_start_position(as5600_t *as5600, uint16_t start_position)
{
    uint8_t write_buffer[] = {AS5600_REG_ZPOS_H, start_position >> 8, start_position};
    i2c_master_transmit(as5600->dev_handle, write_buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_start_position(as5600_t *as5600, uint16_t *start_position)
{
    uint8_t write_buffer[] = {AS5600_REG_ZPOS_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)start_position, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *start_position = (*start_position << 8) | (*start_position >> 8);
}

void as5600_set_stop_position(as5600_t *as5600, uint16_t stop_position)
{
    uint8_t write_buffer[] = {AS5600_REG_MPOS_H, stop_position >> 8, stop_position };
    i2c_master_transmit(as5600->dev_handle, write_buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_stop_position(as5600_t *as5600, uint16_t *stop_position)
{
    uint8_t write_buffer[] = {AS5600_REG_MPOS_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)stop_position, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *stop_position = (*stop_position << 8) | (*stop_position >> 8);
}

void as5600_set_max_angle(as5600_t *as5600, uint16_t max_angle)
{
    uint8_t write_buffer[] = {AS5600_REG_MANG_H, max_angle >> 8, max_angle};
    i2c_master_transmit(as5600->dev_handle, write_buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_max_angle(as5600_t *as5600, uint16_t *max_angle)
{
    uint8_t write_buffer[] = {AS5600_REG_MANG_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)max_angle, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *max_angle = (*max_angle << 8) | (*max_angle >> 8);
}

void as5600_set_conf(as5600_t *as5600, as5600_config_t conf)
{
    uint8_t write_buffer[] = {AS5600_REG_CONF_H, conf.WORD >> 8, conf.WORD};
    i2c_master_transmit(as5600->dev_handle, write_buffer, 3, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_conf(as5600_t *as5600, as5600_config_t *conf)
{
    uint8_t write_buffer[] = {AS5600_REG_CONF_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)&conf->WORD, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    conf->WORD = (conf->WORD << 8) | (conf->WORD >> 8);
}

// -------------------------------------------------------------
// ---------------------- OUTPUT REGISTERS ---------------------
// -------------------------------------------------------------

void as5600_get_raw_angle(as5600_t *as5600, uint16_t *raw_angle)
{
    uint8_t write_buffer[] = {AS5600_REG_RAW_ANGLE_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)raw_angle, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *raw_angle = (*raw_angle << 8) | (*raw_angle >> 8);
}

void as5600_get_angle(as5600_t *as5600, uint16_t *angle)
{
    uint8_t write_buffer[] = {AS5600_REG_ANGLE_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)angle, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *angle = (*angle << 8) | (*angle >> 8);
}

// -------------------------------------------------------------
// ---------------------- STATUS REGISTERS ---------------------
// -------------------------------------------------------------


void as5600_get_status(as5600_t *as5600, uint8_t *status)
{
    uint8_t write_buffer[] = {AS5600_REG_STATUS};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1, status, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_agc(as5600_t *as5600, uint8_t *agc)
{
    uint8_t write_buffer[] = {AS5600_REG_AGC};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1, agc, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void as5600_get_magnitude(as5600_t *as5600, uint16_t *magnitude)
{
    uint8_t write_buffer[] = {AS5600_REG_MAGNITUDE_H};
    i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)magnitude, 2, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    *magnitude = (*magnitude << 8) | (*magnitude >> 8);
}
