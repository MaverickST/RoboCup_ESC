/**
 * \file        bldc_pwm.c
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#include "bldc_pwm.h"

static const char *BLDC_TAG = "bdc_motor_mcpwm";

esp_err_t bldc_init(bldc_pwm_motor_t *motor, uint8_t pwm_gpio_num, uint32_t pwm_freq_hz, uint32_t group_id, uint32_t resolution_hz)
{
    motor->pwm_gpio_num = pwm_gpio_num;
    motor->pwm_freq_hz = pwm_freq_hz;
    motor->group_id = group_id;
    motor->resolution_hz = resolution_hz;
    motor->max_speed_hz = resolution_hz / pwm_freq_hz;
    
    // mcpwm timer
    mcpwm_timer_config_t timer_config = {
        .group_id = group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution_hz,
        .period_ticks = resolution_hz / pwm_freq_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_config, &motor->timer);

    motor->operator = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = group_id,
    };
    mcpwm_new_operator(&operator_config, &motor->operator);

    mcpwm_operator_connect_timer(motor->operator, motor->timer);

    motor->cmp = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(motor->operator, &comparator_config, &motor->cmp);
    mcpwm_comparator_set_compare_value(motor->cmp, 0);

    motor->gen = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pwm_gpio_num,
    };
    mcpwm_new_generator(motor->operator, &generator_config, &motor->gen);

    ///< Set generation actions
    mcpwm_generator_set_actions_on_timer_event(motor->gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(motor->gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmp, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    return ESP_OK;
}

esp_err_t bldc_enable(bldc_pwm_motor_t *motor)
{
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(motor->timer), BLDC_TAG, "enable timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_START_NO_STOP), BLDC_TAG, "start timer failed");
    return ESP_OK;
}

esp_err_t bldc_disable(bldc_pwm_motor_t *motor)
{
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_STOP_EMPTY), BLDC_TAG, "stop timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_disable(motor->timer), BLDC_TAG, "disable timer failed");
    return ESP_OK;
}

esp_err_t bldc_set_speed(bldc_pwm_motor_t *motor, uint32_t speed)
{
    uint32_t nw_cmp = speed*motor->max_speed_hz/1000;
    if (nw_cmp > motor->max_speed_hz) {
        // ESP_LOGE(BLDC_TAG, "speed %d is greater than max speed %d", speed, motor->max_speed_hz);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(motor->cmp, nw_cmp), BLDC_TAG, "set compare value failed");
    return ESP_OK;
}
