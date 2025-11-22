#include "sick_od_mini.h"
#include "app_uart.h"
#include "esp_log.h"

static const char *TAG = "SICK_OD_MINI";

static void sick_od_mini_task(void *arg);
static void sick_od_mini_parse(struct sick_od_mini *self, const uint8_t *data, int len);
static void sick_od_mini_update_distance(sick_od_mini_t *self, float distance_mm);
static uint8_t BCC_XOR(const uint8_t *data);

const uint8_t SICK_OD_MINI_CMD_REQUEST_DATA[] = {0x02, 0x43, 0xB0, 0x01, 0x03, 0xF2};

typedef enum {
    SICK_OD_MINI_STATE_IDLE = 0,
    SICK_OD_MINI_STATE_WAITING_FOR_DATA,
    SICK_OD_MINI_STATE_PROCESSING_DATA,
} SICK_OD_MINI_STATE_e;

esp_err_t sick_od_mini_init( sick_od_mini_t *sensor,
                        const char *task_name,
                        int uart_port                        
                        )
{
    sensor->task_name = task_name;
    sensor->uart_port = uart_port;
    sensor->parse = sick_od_mini_parse;
    sensor->stack = 2048;
    sensor->priority = 5;
    
    sensor->mutex = xSemaphoreCreateMutex();
    if (sensor->mutex == NULL) {
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(sick_od_mini_task, sensor->task_name, sensor->stack, (void*)sensor, sensor->priority, &sensor->task, tskNO_AFFINITY);
    if (sensor->task == NULL) {
        vSemaphoreDelete(sensor->mutex);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void sick_od_mini_task(void *arg)
{
    esp_log_level_set(TAG, ESP_LOG_WARN);
    sick_od_mini_t *self = (sick_od_mini_t *)arg;
    ESP_LOGI(TAG, "Task started for UART port %d", self->uart_port);
    uint8_t buf[64] = {0};
    SICK_OD_MINI_STATE_e state = SICK_OD_MINI_STATE_IDLE;
    int uart_data_len = 0;
    while (1) {
        switch (state)
        {
        case SICK_OD_MINI_STATE_IDLE:
            // Send request to sensor
            self->current_cmd = SICK_OD_MINI_CMD_READ_DATA;
            int l = app_uart_write_bytes(self->uart_port, (const char*)SICK_OD_MINI_CMD_REQUEST_DATA, sizeof(SICK_OD_MINI_CMD_REQUEST_DATA));
            if (l <= 0) {
                // Handle UART write error
                app_uart_flush(self->uart_port);
                ESP_LOGI("TAG", "UART write error");
                return;
            }
            state = SICK_OD_MINI_STATE_WAITING_FOR_DATA;
            break;
        case SICK_OD_MINI_STATE_WAITING_FOR_DATA:
            // Wait for data to be available
            uart_data_len = app_uart_read_bytes(self->uart_port, buf, 256, 100);
            if (uart_data_len > 0)
            {
                state = SICK_OD_MINI_STATE_PROCESSING_DATA;
            }
            else
            {
                state = SICK_OD_MINI_STATE_IDLE;
                ESP_LOGI(TAG, "UART read error");
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            break;
        case SICK_OD_MINI_STATE_PROCESSING_DATA:
            // Parse received data
            self->parse(self, buf, uart_data_len);
            state = SICK_OD_MINI_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        default:
            break;
        }
    }
    vSemaphoreDelete(self->mutex);
    vTaskDelete(NULL);
}

static void sick_od_mini_parse(struct sick_od_mini *self, const uint8_t *data, int len)
{
    uint8_t *_data = (uint8_t *)data;

    for (int i = 0; i < len; i++) {
        if (data[i] == STX) {
            // Found start of frame
            _data = &data[i];
            if (len - i < 6) {
                // Not enough data for a complete frame
                ESP_LOGI(TAG, "Incomplete frame on UART port %d", self->uart_port);
                vTaskDelay(pdMS_TO_TICKS(200));
                return;
            }
            break;
        }
    }
    // Validate frame
    if (_data[0] != STX || 
        _data[1] != ACK || 
        _data[4] != ETX) 
    {   
        ESP_LOGI(TAG, "STX,ACK,ETX mismatch on UART port %d", self->uart_port);
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }
    uint8_t bcc = BCC_XOR(&_data[1]);
    if (bcc != _data[5]) {
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_LOGI(TAG, "BCC mismatch on UART port %d : BCC : 0x%02X", self->uart_port, bcc);
        return;
    }

    // Process distance data
    // Extract distance value
    int16_t raw_value = (_data[2] << 8) | _data[3];
    float distance_mm = raw_value / 100.0f; // convert to mm
    sick_od_mini_update_distance(self, distance_mm);
    ESP_LOGI(TAG, "Distance[%d]: %.2f mm", self->uart_port, distance_mm);
}

static void sick_od_mini_update_distance(sick_od_mini_t *self, float distance_mm)
{
    if (xSemaphoreTake(self->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        self->distance_mm = distance_mm;
        xSemaphoreGive(self->mutex);
    }
}

static uint8_t BCC_XOR(const uint8_t *data)
{
    uint8_t bcc = 0;
    for (int i = 0; i < 3; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

static void sick_od_mini_print_buffer(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        ESP_LOGI(TAG, "data[%d]: 0x%02X", i, data[i]);
    }
}