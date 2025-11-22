#ifndef __MAIN_PULSE_COUNTER_H__
#define __MAIN_PULSE_COUNTER_H__
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

typedef struct {
    QueueHandle_t pcnt_val_queue;
    SemaphoreHandle_t mutex;
    TaskHandle_t task;
    const char *task_name;
    size_t stack;
    UBaseType_t priority;
    
    int channel_a_gpio;
    int channel_b_gpio;
    int high_limit;
    int low_limit;
    int intr_priority;

    pcnt_watch_event_data_t event_data;
    pcnt_unit_config_t unit_config;
    pcnt_unit_handle_t pcnt_unit;

    float distance_x;
} encoder_t;

void pulse_counter_init(encoder_t *self);
float pulse_counter_get_distance_x(encoder_t *self);
#endif