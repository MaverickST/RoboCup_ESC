/**
 * \file        as5600_types.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

/**
 * @brief Register addresses
 * 
 */
typedef enum 
{
    AS5600_REG_ZMCO = 0x00,     ///< ZMCO shows how many times ZPOS and MPOS have been permanently written.
    AS5600_REG_ZPOS_H = 0x01,     /*!< Zero-Position Offset */
    AS5600_REG_ZPOS_L = 0x02,     /*!< Zero-Position Offset */
    AS5600_REG_MPOS_H = 0x03,     /*!< Magnet Position */
    AS5600_REG_MPOS_L = 0x04,     /*!< Magnet Position */
    AS5600_REG_MANG_H = 0x05,     /*!< Magnet Angle */
    AS5600_REG_MANG_L = 0x06,     /*!< Magnet Angle */
    AS5600_REG_CONF_H = 0x07,     /*!< Configuration */
    AS5600_REG_CONF_L = 0x08,     /*!< Configuration */
    AS5600_REG_STATUS = 0x0B,     /*!< Status */
    AS5600_REG_RAW_ANGLE_H = 0x0C,     /*!< Raw Angle */
    AS5600_REG_RAW_ANGLE_L = 0x0D,     /*!< Raw Angle */
    AS5600_REG_ANGLE_H = 0x0E,     /*!< Angle */
    AS5600_REG_ANGLE_L = 0x0F,     /*!< Angle */
    AS5600_REG_AGC = 0x1A,     /*!< Automatic Gain Control */
    AS5600_REG_MAGNITUDE_H = 0x1B,     /*!< Magnitude */
    AS5600_REG_MAGNITUDE_L = 0x1C,     /*!< Magnitude */
    AS5600_REG_BURN = 0xFF     /*!< Burn */
} as5600_reg_t;


/**
 * @brief Power modes for PM bitfield at the CONF register
 * 
 */
typedef enum 
{
    AS5600_POWER_MODE_NOM = 0x00,    /*!< Normal mode */
    AS5600_POWER_MODE_LPM1 = 0x01,   /*!< Low power mode 1 */
    AS5600_POWER_MODE_LPM2 = 0x02,   /*!< Low power mode 2 */
    AS5600_POWER_MODE_LPM3 = 0x03,   /*!< Low power mode 3 */
    AS5600_POWER_MODE_COUNT = 0x04   /*!< Number of power modes */
} as5600_power_mode_t;


/**
 * @brief PWM frequency for PW bitfield at the CONF register
 * 
 */
typedef enum
{
    AS5600_HYSTERESIS_OFF = 0x00,   /*!< Hysteresis off */
    AS5600_HYSTERESIS_1LSB = 0x01,  /*!< Hysteresis 1LSB */
    AS5600_HYSTERESIS_2LSB = 0x02,  /*!< Hysteresis 2LSB */
    AS5600_HYSTERESIS_3LSB = 0x03,  /*!< Hysteresis 3LSB */
    AS5600_HYSTERESIS_COUNT = 0x04  /*!< Number of hysteresis modes */
} as5600_hysteresis_t;


typedef struct
{
    as5600_reg_t reg;
    uint8_t data;
} as5600_config_t;