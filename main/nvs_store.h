#pragma once
#include <stdbool.h>
#include "esp_bt_defs.h"   // esp_bd_addr_t

bool nvs_load_host_bda(esp_bd_addr_t out_bda);
void nvs_save_host_bda(const esp_bd_addr_t bda);
void nvs_erase_host_bda(void);
