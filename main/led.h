#pragma once
#include <stdint.h>

typedef enum {
    LED_BOOTING = 0,
    LED_PAIRING,
    LED_CONNECTABLE_WAIT,
    LED_CONNECTED,
    LED_FACTORY_RESET,
} led_state_t;

void led_init(void);
void led_set_state(led_state_t st);
void led_task_start(void);   // starts the FreeRTOS task
