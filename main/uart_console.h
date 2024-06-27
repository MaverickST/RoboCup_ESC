/**
 * \file        uart_console.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __UART_CONSOLE_H__
#define __UART_CONSOLE_H__

#include <stdio.h>
#include "esp_log.h"
#include "soc/soc.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "hal/uart_hal.h"

#include "types.h"


#define UART_NUM        0
/* Notice that ESP32 has to use the iomux input to configure uart as wakeup source
 * Please use 'UxRXD_GPIO_NUM' as uart rx pin. No limitation to the other target */
#define UART_TX_IO_NUM  U0TXD_GPIO_NUM
#define UART_RX_IO_NUM  U0RXD_GPIO_NUM

#define UART_WAKEUP_THRESHOLD   2

#define READ_BUF_SIZE   128
#define UART_BUF_SIZE   (READ_BUF_SIZE * 2)

// static QueueHandle_t uart_evt_que = NULL;


/**
 * @brief Structure to handle the UART console.
 * It just will just receive the data from the UART, but it can be extended to send data.
 * 
 */
typedef struct
{
    uint8_t *data;
    uint16_t len;
    uint8_t uart_num;

}uart_console_t;


/**
 * @brief Initialize the UART console
 * 
 */
void uconsole_init(uart_console_t *uc);

/**
 * @brief Read the data from the UART console
 * 
 * @param uc 
 */
void uconsole_read_data(uart_console_t *uc);

/**
 * @brief Deinitialize the UART console
 * 
 * @param uc 
 */
void uconsole_deinit(uart_console_t *uc);

/**
 * @brief UART console interrupt handler
 * 
 * @param arg 
 */
void uconsole_intr_handler(void *arg);

#endif // __UART_CONSOLE_H__