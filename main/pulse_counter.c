#include "pulse_counter.h"
#include <math.h>

#define TAG "pulse_counter"

static volatile int pulse_count = 0;
static volatile int encoder_revolution = 0;

#define EXAMPLE_PCNT_HIGH_LIMIT 600
#define EXAMPLE_PCNT_LOW_LIMIT  -600
#define ENCODER_PPR  600  // pulses per revolution
#define GEAR_RATIO   1    // gear ratio
#define WHEEL_DIA_MM  200   // wheel diameter in mm
#define WHEEL_CIRCUM_MM  (WHEEL_DIA_MM * M_PI)

#define EXAMPLE_EC11_GPIO_A 16
#define EXAMPLE_EC11_GPIO_B 17

static void pulse_counter_task(void *arg);
static bool pulse_counter_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
static void pulse_counter_set_distance_x(encoder_t *self, float distance_x);

static float pulse_counter_calculate_current_x_m(float begin_pos, int pulse_count)
{
    float distance_x = 0.0f;
    // total revolutions = full revolutions + partial revolution
    distance_x = (encoder_revolution + (float)pulse_count / ENCODER_PPR) * WHEEL_CIRCUM_MM / 1000.0f; // convert to cm
    return begin_pos + distance_x;
}

static bool pulse_counter_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void pulse_counter_init(encoder_t *self)
{
    self->pcnt_val_queue = xQueueCreate(1, sizeof(int));
    self->mutex = xSemaphoreCreateMutex();
    self->task_name = "pulse_counter_task";
    self->stack = 2048;
    self->priority = 10;
    self->channel_a_gpio = EXAMPLE_EC11_GPIO_A;
    self->channel_b_gpio = EXAMPLE_EC11_GPIO_B;
    self->high_limit = EXAMPLE_PCNT_HIGH_LIMIT;
    self->low_limit = EXAMPLE_PCNT_LOW_LIMIT;
    self->intr_priority = 1;

    // Initialization code for pulse counter can be added here
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };

    self->pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &self->pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(self->pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(self->pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(self->pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    
    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, 0, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(self->pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = pulse_counter_on_reach,
    };

    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(self->pcnt_unit, &cbs, self->pcnt_val_queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(self->pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(self->pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(self->pcnt_unit));


    xTaskCreatePinnedToCore(pulse_counter_task, self->task_name, 2048, (void*)self, 10, &self->task, 1);
}

static void pulse_counter_task(void *arg)
{
    encoder_t* self = (encoder_t* )arg;
    int event_count = 0;
    while (1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(self->pcnt_unit, &pulse_count));
        if (xQueueReceive(self->pcnt_val_queue, &event_count, pdMS_TO_TICKS(10)) == pdTRUE) 
        {
            if (event_count == EXAMPLE_PCNT_HIGH_LIMIT) {
                encoder_revolution += 1;
            } else if (event_count == EXAMPLE_PCNT_LOW_LIMIT) {
                encoder_revolution -= 1;
            }
            // ESP_LOGI(TAG, "PCNT event: count=%d, revolution=%d", event_count, encoder_revolution);
        }
        float _distance_x = pulse_counter_calculate_current_x_m(0.0f, pulse_count);
        pulse_counter_set_distance_x(self, _distance_x);
        /* ESP_LOGI(TAG, "Distance X: %.2f m", distance_x); */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void pulse_counter_set_distance_x(encoder_t *self, float distance_x)
{
    if (self->mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(self->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        self->distance_x = distance_x;
        xSemaphoreGive(self->mutex);
    }
}

float pulse_counter_get_distance_x(encoder_t *self)
{
    if (self->mutex == NULL) {
        return 0.0f;
    }

    float distance_x = 0.0f;
    if (xSemaphoreTake(self->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        distance_x = self->distance_x;
        xSemaphoreGive(self->mutex);
    }
    return distance_x;
}

