#include "app_uart.h"

void app_uart_init(int uart_port, int tx_pin, int rx_pin, int baudrate)
{
    uart_config_t cfg = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(uart_port, &cfg);
    uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_port, 256, 256, 0, NULL, 0);
}

void app_uart_deinit(int uart_port)
{
    uart_driver_delete(uart_port);
}

void app_uart_flush(int uart_port)
{
    uart_flush(uart_port);
}

int app_uart_write_bytes(int uart_port, const char* data, size_t length)
{
    return uart_write_bytes(uart_port, data, length);
}

int app_uart_read_bytes(int uart_port, void* buf, size_t length, uint32_t ms_to_wait)
{
   return uart_read_bytes(uart_port, buf, length, pdMS_TO_TICKS(ms_to_wait));
}
