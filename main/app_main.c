#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "led.h"
#include "buttons.h"
#include "bt_hid.h"

static const char *TAG = "APP";

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    led_init();
    led_task_start();

    buttons_init();
    buttons_task_start();

    bt_hid_init();

    ESP_LOGI(TAG, "Ready. Hold VOL+ + VOL- for 3s to pair, 10s to factory reset.");
}
