#include "sdkconfig.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/uart.h"
#include "sick_od_mini.h"
#include "app_uart.h"
#include "pulse_counter.h"

#define print_info ESP_LOGI("MAIN", "Encoder %.2f m \t Sick-1 : %.2f \t Sick-2 : %.2f", pulse_counter_get_distance_x(&encoder_x), sick_sensor[0].distance_mm, sick_sensor[1].distance_mm);

sick_od_mini_t sick_sensor[2] = {0};
encoder_t encoder_x = {0};


void app_main(void)
{
    // Your application code here
    app_uart_init(UART_NUM_1, 26, 13, 115200);
    app_uart_init(UART_NUM_2, 19, 18, 115200);

    sick_od_mini_init(&sick_sensor[0], "SICK_OD_MINI_1", UART_NUM_1);
    sick_od_mini_init(&sick_sensor[1], "SICK_OD_MINI_2", UART_NUM_2);

    pulse_counter_init(&encoder_x);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

