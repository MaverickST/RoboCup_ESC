/**
 * \file        bldc_pwm.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __PWM_MOTOR_H__
#define __PWM_MOTOR_H__


#include <stdint.h>
#include <stdbool.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

typedef struct {
    uint32_t pwm_gpio_num;  ///< GPIO number
    uint32_t pwm_freq_hz;   ///< PWM frequency in Hz
    uint32_t pwm_duty_us;   ///< PWM duty cycle in microseconds
    uint32_t max_speed_hz;  ///< Maximum speed in Hz

    int group_id;           ///< MCPWM group number 
    uint32_t resolution_hz; ///< MCPWM timer frequency

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t cmp;
    mcpwm_gen_handle_t gen;

} bldc_pwm_motor_t;

/**
 * @brief Initialize the BLDC motor
 * 
 * @param motor instance of the motor
 * @param pwm_gpio_num 
 * @param pwm_freq_hz 
 * @param group_id 
 * @param resolution_hz 
 */
esp_err_t bldc_init(bldc_pwm_motor_t *motor, uint8_t pwm_gpio_num, uint32_t pwm_freq_hz, uint32_t group_id, uint32_t resolution_hz);

/**
 * @brief Enable the motor
 * 
 * @param motor instance of the motor
 */
esp_err_t bldc_enable(bldc_pwm_motor_t *motor);

/**
 * @brief Disable the motor
 * 
 * @param motor instance of the motor
 */
esp_err_t bldc_disable(bldc_pwm_motor_t *motor);

/**
 * @brief Set the speed of the motor
 * 
 * @param motor instance of the motor
 * @param speed range from 0 to 1000, where 0 is stopped and 1000 is maximum speed
 */
esp_err_t bldc_set_speed(bldc_pwm_motor_t *motor, uint32_t speed);

#endif // __PWM_MOTOR_H__