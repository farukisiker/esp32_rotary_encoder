/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/uart.h"

static void uart1_task_handler(void *arg);
static void uart2_task_handler(void *arg);
static void log_handler(void *arg);

static const char *TAG = "example";

static int pulse_count = 0;
static int encoder_revolution = 0;
static float distance_x = 0.0f;

#define EXAMPLE_PCNT_HIGH_LIMIT 600
#define EXAMPLE_PCNT_LOW_LIMIT  -600 
#define ENCODER_PPR  600  // pulses per revolution
#define GEAR_RATIO   1    // gear ratio
#define WHEEL_DIA_MM  200   // wheel diameter in mm
#define WHEEL_CIRCUM_MM  (WHEEL_DIA_MM * 3.1416)

#define EXAMPLE_EC11_GPIO_A 16
#define EXAMPLE_EC11_GPIO_B 17

TaskHandle_t log_handle = NULL;
TaskHandle_t uart1_task_handle = NULL;
TaskHandle_t uart2_task_handle = NULL;
QueueHandle_t uart1_queue = NULL;
QueueHandle_t uart2_queue = NULL;
QueueHandle_t pcnt_val_queue = NULL;
SemaphoreHandle_t pcnt_semaphore = NULL;

static esp_err_t uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_26, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0));
    
    xTaskCreatePinnedToCore(uart1_task_handler, "uart1_task", 2048, (void*)UART_NUM_1, 10, &uart1_task_handle, 1);

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 19, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 256, 0, 0, NULL, 0));
    
    xTaskCreatePinnedToCore(uart2_task_handler, "uart2_task", 2048, (void*)UART_NUM_2, 10, &uart2_task_handle, 1);
    
    // test print
    // uart_write_bytes(UART_NUM_0, "UART initialized\n", 17);
    return ESP_OK;
}

float calculate_current_x_m(float begin_pos, int pulse_count)
{
    float distance_x = 0.0f;
    // total revolutions = full revolutions + partial revolution
    distance_x = (encoder_revolution + (float)pulse_count / ENCODER_PPR) * WHEEL_CIRCUM_MM / 1000.0f; // convert to cm
    return begin_pos + distance_x;
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void app_main(void)
{
    pcnt_val_queue = xQueueCreate(1, sizeof(float));
    uart1_queue = xQueueCreate(1, sizeof(float));
    uart2_queue = xQueueCreate(1, sizeof(float));
    pcnt_semaphore = xSemaphoreCreateMutex();
    if (!pcnt_val_queue || !uart1_queue || !uart2_queue || !pcnt_semaphore) {
        ESP_LOGE(TAG, "Failed to create queues or semaphore");
        return;
    }
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, 0, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    
    xTaskCreatePinnedToCore(log_handler, "log_task", 4096, (void*)pcnt_unit, 10, &log_handle, 1);

    ESP_LOGI(TAG, "initialize uart");
    ESP_ERROR_CHECK(uart_init());


#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif
    // Report counter value
    int event_count = 0;
    float begin_pos_x = 0.0f;
    while (1) {
        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(20)))
        {
            //ESP_LOGI(TAG, "PCNT event: count=%d", event_count);
            if (event_count == EXAMPLE_PCNT_HIGH_LIMIT) {
                encoder_revolution += 1;
                //ESP_LOGI(TAG, "Reached high limit");
            } else if (event_count == EXAMPLE_PCNT_LOW_LIMIT) {
                encoder_revolution -= 1;
                //ESP_LOGI(TAG, "Reached low limit");
            }
        }
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        begin_pos_x = calculate_current_x_m(0, pulse_count);
        xQueueSend(pcnt_val_queue, &begin_pos_x, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void uart1_task_handler(void *arg)
{
    /*
    Transmission command: STX (02h) C (43h) B0h 01h ETX (03h) BCC (F2h)
    Incoming data: STX (02h) ACK (06h) FCh 6Fh ETX (03h) BCC (95h)
    *example with OD1-B035x15xxx/ measuring value = –9.13 mm
    */
    const uart_port_t uart_num = (uart_port_t)arg;
    const static uint8_t sick_sensor_ask[] = {0x02, 0x43, 0xB0, 0x01, 0x03, 0xF2};
    float distance_mm_z = 0.0f;
    uint8_t* data = (uint8_t*) malloc(10);
    
    while (1) {
        uart_write_bytes(uart_num, (const char*)sick_sensor_ask, sizeof(sick_sensor_ask));
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, 10, pdMS_TO_TICKS(100));
        if (len > 0) {
            // get distance value from received data
            if (len >= 6 && data[0] == 0x02 && data[1] == 0x06) {
                // valid data frame
                int16_t raw_value = (data[2] << 8) | data[3];
                distance_mm_z = raw_value / 100.0f; // convert to mm
                xQueueSend(uart1_queue, &distance_mm_z, portMAX_DELAY);
                uart_flush(uart_num);
            }
        }
        else {
            ESP_LOGW(TAG, "No data received from sensor");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

static void uart2_task_handler(void *arg)
{
    /*
    Transmission command: STX (02h) C (43h) B0h 01h ETX (03h) BCC (F2h)
    Incoming data: STX (02h) ACK (06h) FCh 6Fh ETX (03h) BCC (95h)
    *example with OD1-B035x15xxx/ measuring value = –9.13 mm
    */
    const uart_port_t uart_num = (uart_port_t)arg;
    const static uint8_t sick_sensor_ask[] = {0x02, 0x43, 0xB0, 0x01, 0x03, 0xF2};
    float distance_mm_z = 0.0f;
    uint8_t* data = (uint8_t*) malloc(10);
    
    while (1) {
        uart_write_bytes(uart_num, (const char*)sick_sensor_ask, sizeof(sick_sensor_ask));
        // Read data from UART
        int len = uart_read_bytes(uart_num, data, 10, pdMS_TO_TICKS(100));
        if (len > 0) {
            // get distance value from received data
            if (len >= 6 && data[0] == 0x02 && data[1] == 0x06) {
                // valid data frame
                int16_t raw_value = (data[2] << 8) | data[3];
                distance_mm_z = raw_value / 100.0f; // convert to mm
                xQueueSend(uart2_queue, &distance_mm_z, portMAX_DELAY);
                uart_flush(uart_num);
            }
        }
        else {
            ESP_LOGW(TAG, "No data received from sensor");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

static void log_handler(void *arg)
{
    float pcnt_value = 0;
    float distance_mm_z[2] = {0.0f, 0.0f};
    while (1) {
        xQueueReceive(pcnt_val_queue, &pcnt_value, portMAX_DELAY);
        xQueueReceive(uart1_queue, &distance_mm_z[0], portMAX_DELAY);
        xQueueReceive(uart2_queue, &distance_mm_z[1], portMAX_DELAY);
        ESP_LOGI(TAG, "PS: %.2f, Dist[1]: %.2f , Dist[2]: %.2f ", pcnt_value, distance_mm_z[0], distance_mm_z[1]);
    }
}