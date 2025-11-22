#ifndef __SICK_OD_MINI_H__
#define __SICK_OD_MINI_H__
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define STX ((uint8_t)(0x02))
#define ETX (uint8_t)0x03
#define ACK (uint8_t)0x06
#define NACK (uint8_t)0x15

typedef enum SICK_OD_MINI_CMD{
    SICK_OD_MINI_CMD_READ_DATA = 0x43,
    SICK_OD_MINI_CMD_READ_SETTINGS = 0x52,
    SICK_OD_MINI_CMD_WRITE_SETTINGS = 0x57,
} SICK_OD_MINI_CMD_e;


typedef struct sick_od_mini {
    const char *task_name;
    int uart_port;

    TaskHandle_t task;
    int stack;
    int priority;
    SemaphoreHandle_t mutex;

    // Hepsi aynı sensör tipi → hepsi aynı fonksiyonları kullanır
    void (*parse)(struct sick_od_mini *self, const uint8_t *, int);

    SICK_OD_MINI_CMD_e current_cmd;
    float distance_mm;
} sick_od_mini_t;

typedef void (*parse)(struct sick_od_mini *self, const uint8_t *, int);

esp_err_t sick_od_mini_init( sick_od_mini_t *sensor,
                        const char *name,
                        int uart_port                        
                        );


#endif // __SICK_OD_MINI_H__