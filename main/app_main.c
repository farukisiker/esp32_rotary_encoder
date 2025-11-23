#include "sdkconfig.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/uart.h"
#include "sick_od_mini.h"
#include "app_uart.h"
#include "pulse_counter.h"
#include "app_ble.h"

#define print_info ESP_LOGI("MAIN", "Encoder %.2f m \t Sick-1 : %.2f \t Sick-2 : %.2f", pulse_counter_get_distance_x(&encoder_x), sick_sensor[0].distance_mm, sick_sensor[1].distance_mm);

sick_od_mini_t sick_sensor[2] = {0};
encoder_t encoder_x = {0};


void app_main(void)
{
    /* NVS'yi başlat - BLE için gerekli */
    int rc;
    rc = nvs_flash_init();
    if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(rc);

    // Your application code here
    app_uart_init(UART_NUM_1, 26, 13, 115200);
    app_uart_init(UART_NUM_2, 19, 18, 115200);

    sick_od_mini_init(&sick_sensor[0], "SICK_OD_MINI_1", UART_NUM_1);
    sick_od_mini_init(&sick_sensor[1], "SICK_OD_MINI_2", UART_NUM_2);

    pulse_counter_init(&encoder_x);

    app_ble_init();
    while (1) {
        app_ble_update_values(pulse_counter_get_distance_x(&encoder_x), 
                              sick_sensor[0].distance_mm, 
                              sick_sensor[1].distance_mm);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

