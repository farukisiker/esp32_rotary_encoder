/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef MAIN_APP_UART_H_
#define MAIN_APP_UART_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

void app_uart_init(int uart_port, int tx_pin, int rx_pin, int baudrate);
void app_uart_deinit(int uart_port);
void app_uart_flush(int uart_port);
int app_uart_write_bytes(int uart_port, const char* data, size_t length);
int app_uart_read_bytes(int uart_port, void* buf, size_t length, uint32_t ms_to_wait);

#endif /* MAIN_APP_UART_H_ */