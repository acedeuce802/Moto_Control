#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static inline void delay_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks < 1) ticks = 1;
    vTaskDelay(ticks);
}