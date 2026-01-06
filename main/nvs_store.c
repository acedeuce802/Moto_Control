#include "nvs_store.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define NVS_NS       "moto"
#define NVS_KEY_HOST "host_bda"

static const char *TAG = "NVS_STORE";

/* ---------------- NVS ---------------- */
#define NVS_NS "moto"
#define NVS_KEY_HOST "host_bda"   // 6 bytes

/* ---------------- NVS helper ---------------- */
void nvs_save_host_bda(const esp_bd_addr_t bda)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_HOST, bda, ESP_BD_ADDR_LEN);
    nvs_commit(h);
    nvs_close(h);
}

bool nvs_load_host_bda(esp_bd_addr_t out_bda)
{
    nvs_handle_t h;
    size_t len = ESP_BD_ADDR_LEN;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_HOST, out_bda, &len);
    nvs_close(h);
    return (err == ESP_OK && len == ESP_BD_ADDR_LEN);
}

void nvs_erase_host_bda(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY_HOST);
    nvs_commit(h);
    nvs_close(h);
}