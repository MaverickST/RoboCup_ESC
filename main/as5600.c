#include "as5600.h"

void as5600_init(as5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda)
{
    as5600->i2c_num = i2c_num;
    as5600->scl = scl;
    as5600->sda = sda;

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
    if (!as5600_is_valid_reg(as5600, reg)) {
        ESP_LOGI("AS5600", "Invalid register");
        return;
    }
    uint8_t write_buffer[] = {reg};
    printf("reg: %02x\n", *write_buffer);
    switch (reg)
    {
    case AS5600_REG_ZMCO:
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 1, 5000 / portTICK_PERIOD_MS);
        break;
    case AS5600_REG_STATUS:
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 1, 5000 / portTICK_PERIOD_MS);
        break;
    case AS5600_REG_AGC:
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 1, 5000 / portTICK_PERIOD_MS);
        break;
    default:
        i2c_master_transmit_receive(as5600->dev_handle, write_buffer, 1,(uint8_t *)data, 2, 5000 / portTICK_PERIOD_MS);
        *data = (*data << 8) | (*data >> 8);
        break;
    }
}

void as5600_write_reg(as5600_t *as5600, as5600_reg_t reg, uint16_t data)
{
}

bool as5600_is_valid_reg(as5600_t *as5600, as5600_reg_t reg)
{
    if (reg == AS5600_REG_ZMCO || reg == AS5600_REG_ZPOS_H || reg == AS5600_REG_ZPOS_L || 
        reg == AS5600_REG_MPOS_H || reg == AS5600_REG_MPOS_L || reg == AS5600_REG_MANG_H || 
        reg == AS5600_REG_MANG_L || reg == AS5600_REG_CONF_H || reg == AS5600_REG_CONF_L || 
        reg == AS5600_REG_STATUS || reg == AS5600_REG_RAW_ANGLE_H || reg == AS5600_REG_RAW_ANGLE_L || 
        reg == AS5600_REG_ANGLE_H || reg == AS5600_REG_ANGLE_L || reg == AS5600_REG_AGC || 
        reg == AS5600_REG_MAGNITUDE_H || reg == AS5600_REG_MAGNITUDE_L || reg == AS5600_REG_BURN)
    {
        return true;
    }
    return false;
}

// esp_err_t i2c_master_write_read(i2c_port_t i2c_num, uint8_t reg, uint8_t *data_rd, uint8_t size)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, AS5600_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

//     for (int i = 0; i < size; i++) {
//         if (i == size - 1) {
//             i2c_master_read_byte(cmd, data_rd + i, NACK_VAL);
//         }
//         else {
//             i2c_master_read_byte(cmd, data_rd + i, ACK_VAL);
//         }
//     }

//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }
