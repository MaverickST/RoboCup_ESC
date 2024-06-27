#include "uart_console.h"

void uconsole_init(uart_console_t *uc)
{
    uc->data = NULL;
    uc->len = 0;
    uc->uart_num = UART_NUM;

    ///< Configure the UART
    uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = UART_WAKEUP_THRESHOLD,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL,  
                        ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_LEVEL3));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // uart_set_pin(UART_NUM, UART_TX_IO_NUM, UART_RX_IO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uc->data = (uint8_t *)malloc(READ_BUF_SIZE);
    uc->len = 0;

    ///< Configure the UART interrupt configuration
    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_INTR_RXFIFO_FULL,
        .rxfifo_full_thresh = UART_WAKEUP_THRESHOLD,
        .rx_timeout_thresh = UART_WAKEUP_THRESHOLD*2,
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM)); ///< Enable the RX interrupt
    esp_intr_alloc(ETS_UART0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_LEVEL3, 
                    uconsole_intr_handler, uc, NULL);
}

void uconsole_read_data(uart_console_t *uc)
{
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, (size_t*)&uc->len));
    uc->len = uart_read_bytes(UART_NUM, uc->data, uc->len, 100);
    uart_flush(UART_NUM);
}

void uconsole_deinit(uart_console_t *uc)
{
    free(uc->data);
    uc->data = NULL;
    uc->len = 0;
    uc->uart_num = UART_NUM;

    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM));
}

void uconsole_intr_handler(void *arg)
{
    uart_console_t *uc = (uart_console_t *)arg;

    uconsole_read_data(uc);
    ESP_LOGI("UART", "Data received: %s", uc->data);
    gFlag.uc_data = true;
}
