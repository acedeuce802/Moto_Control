#include "bt_hid.h"

#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidd_api.h"

#include "timeutil.h"
#include "led.h"
#include "nvs_store.h"
#include "config.h"

static const char *TAG = "BT_HID";

static volatile bool s_hid_connected = false;
static volatile bool s_pairing_mode  = false;
static volatile bool s_hid_ready     = false;

bool bt_hid_is_connected(void) { return s_hid_connected; }

/* ---------------- HID Report Map (Consumer Control, 16 bits) ----------------
   Report ID 1:
     bit0  Volume Increment (0xE9)
     bit1  Volume Decrement (0xEA)
     bit2  Scan Next Track  (0xB5)
     bit3  Play/Pause       (0xCD)
     bit4  Pause            (0xB1)
     bit5  Voice Command    (0xCF)  (best-effort on Android)
*/
static const uint8_t s_hid_report_map[] = {
    0x05, 0x0C,             // USAGE_PAGE (Consumer)
    0x09, 0x01,             // USAGE (Consumer Control)
    0xA1, 0x01,             // COLLECTION (Application)
    0x85, RID_CONSUMER,     //   REPORT_ID (1)
    0x15, 0x00,             //   LOGICAL_MINIMUM (0)
    0x25, 0x01,             //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,             //   REPORT_SIZE (1)
    0x95, 0x10,             //   REPORT_COUNT (16)

    0x09, 0xE9,             //   USAGE (Volume Increment)  bit0
    0x09, 0xEA,             //   USAGE (Volume Decrement)  bit1
    0x09, 0xB5,             //   USAGE (Scan Next Track)   bit2
    0x09, 0xCD,             //   USAGE (Play/Pause)        bit3
    0x09, 0xB1,             //   USAGE (Pause)             bit4
    0x09, 0xCF,             //   USAGE (Voice Command)     bit5

    // pad remaining 10 bits
    0x09, 0x00, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00,
    0x09, 0x00, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00,
    0x09, 0x00, 0x09, 0x00,

    0x81, 0x02,             //   INPUT (Data,Var,Abs)
    0xC0                    // END_COLLECTION
};

/* ---------- Forward declarations ---------- */
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

static void bt_set_pairing_mode(bool enable)
{
    // connectable always; discoverable only when pairing
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                             enable ? ESP_BT_GENERAL_DISCOVERABLE : ESP_BT_NON_DISCOVERABLE);
}

void bt_hid_enter_pairing_mode(void)
{
    if (s_pairing_mode) return;

    s_pairing_mode = true;
    ESP_LOGI(TAG, "Entering pairing mode (discoverable/connectable)...");
    led_set_state(LED_PAIRING);
    bt_set_pairing_mode(true);
}

/* ---------- Bonds ---------- */
static void clear_all_bonds(void)
{
    int dev_num = esp_bt_gap_get_bond_device_num();
    if (dev_num <= 0) {
        ESP_LOGI(TAG, "No bonded devices to clear.");
        return;
    }

    esp_bd_addr_t *dev_list = (esp_bd_addr_t *)calloc(dev_num, sizeof(esp_bd_addr_t));
    if (!dev_list) {
        ESP_LOGE(TAG, "Failed to allocate bond list.");
        return;
    }

    if (esp_bt_gap_get_bond_device_list(&dev_num, dev_list) == ESP_OK) {
        for (int i = 0; i < dev_num; i++) {
            esp_err_t err = esp_bt_gap_remove_bond_device(dev_list[i]);
            ESP_LOGW(TAG, "Remove bond %d/%d: %s", i + 1, dev_num, esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to get bond device list.");
    }

    free(dev_list);
}

void bt_hid_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing bonds + saved host, then reboot.");
    led_set_state(LED_FACTORY_RESET);
    delay_ms(250);

    nvs_erase_host_bda();
    clear_all_bonds();
    delay_ms(200);

    esp_restart();
}

/* ---------- Optional: bond list helper (single-host assumption) ---------- */
static bool get_first_bonded_host(esp_bd_addr_t out_bda)
{
    int num = esp_bt_gap_get_bond_device_num();
    if (num <= 0) return false;

    esp_bd_addr_t *list = (esp_bd_addr_t *)calloc(num, sizeof(esp_bd_addr_t));
    if (!list) return false;

    int n = num;
    esp_err_t err = esp_bt_gap_get_bond_device_list(&n, list);
    if (err == ESP_OK && n > 0) {
        memcpy(out_bda, list[0], ESP_BD_ADDR_LEN);
        free(list);
        return true;
    }

    free(list);
    return false;
}

/* ---------- Autoconnect task ---------- */
static void hid_autoconnect_task(void *arg)
{
    delay_ms(1200);

    if (s_hid_connected) { vTaskDelete(NULL); return; }

    esp_bd_addr_t bda;
    if (!nvs_load_host_bda(bda)) {
        ESP_LOGI(TAG, "No saved host address; autoconnect skipped.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Attempting HID autoconnect to %02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    for (int i = 0; i < 5 && !s_hid_connected; i++) {
        esp_err_t err = esp_bt_hid_device_connect(bda);
        ESP_LOGI(TAG, "esp_bt_hid_device_connect attempt %d -> %s", i + 1, esp_err_to_name(err));
        delay_ms(1500 + i * 500);
    }

    vTaskDelete(NULL);
}

/* ---------- GAP callback ---------- */
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Auth success: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGW(TAG, "Auth failed, status=0x%x", param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "SSP confirm request, numeric=%" PRIu32, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        default:
            break;
    }
}

/* ---------- HID callback ---------- */
static void hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
        case ESP_HIDD_INIT_EVT:
            ESP_LOGI(TAG, "HID init status=%d", param->init.status);
            break;

        case ESP_HIDD_REGISTER_APP_EVT:
            ESP_LOGI(TAG, "HID register app status=%d", param->register_app.status);
            s_hid_ready = (param->register_app.status == ESP_OK);
            break;

        case ESP_HIDD_OPEN_EVT: {
            ESP_LOGI(TAG, "HID connected");
            s_hid_connected = true;
            led_set_state(LED_CONNECTED);

            // Save host address (single-host): use bond list to avoid struct field drift
            esp_bd_addr_t bda;
            if (get_first_bonded_host(bda)) {
                nvs_save_host_bda(bda);
                ESP_LOGI(TAG, "Saved bonded host as last host.");
            }

            s_pairing_mode = false;
            bt_set_pairing_mode(false);
            break;
        }

        case ESP_HIDD_CLOSE_EVT:
            ESP_LOGI(TAG, "HID disconnected");
            s_hid_connected = false;

            if (s_pairing_mode) {
                led_set_state(LED_PAIRING);
                bt_set_pairing_mode(true);
            } else {
                led_set_state(LED_CONNECTABLE_WAIT);
                bt_set_pairing_mode(false);
            }
            break;

        default:
            break;
    }
}

void bt_hid_init(void)
{
    static bool s_bt_inited = false;
    if (s_bt_inited) {
        ESP_LOGW(TAG, "BT already initialized; skipping");
        return;
    }
    s_bt_inited = true;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("MotoControls"));

    // SSP Just Works
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap)));

    ESP_ERROR_CHECK(esp_bt_hid_device_register_callback(hidd_cb));
    ESP_ERROR_CHECK(esp_bt_hid_device_init());

    esp_hidd_app_param_t app_param = {
        .name = "MotoControls",
        .description = "Motorcycle phone controls",
        .provider = "DIY",
        .subclass = 0x40,
        .desc_list = (uint8_t *)s_hid_report_map,
        .desc_list_len = sizeof(s_hid_report_map),
    };

    static esp_hidd_qos_param_t s_qos_in  = { 0 };
    static esp_hidd_qos_param_t s_qos_out = { 0 };
    ESP_ERROR_CHECK(esp_bt_hid_device_register_app(&app_param, &s_qos_in, &s_qos_out));

    int bonds = esp_bt_gap_get_bond_device_num();
    ESP_LOGI(TAG, "Bonded devices: %d", bonds);

    if (bonds == 0) {
        led_set_state(LED_PAIRING);
        ESP_LOGI(TAG, "No bonds; enabling pairing mode.");
        bt_hid_enter_pairing_mode();
    } else {
        led_set_state(LED_CONNECTABLE_WAIT);
        bt_set_pairing_mode(false);
        xTaskCreatePinnedToCore(hid_autoconnect_task, "hid_autoconnect", 4096, NULL, 3, NULL, 0);
    }
}

void bt_hid_send_consumer(uint16_t bits)
{
    if (!s_hid_ready || !s_hid_connected) return;

    uint8_t report[2] = {
        (uint8_t)(bits & 0xFF),
        (uint8_t)((bits >> 8) & 0xFF)
    };

    // Press
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA,
                                  RID_CONSUMER,
                                  sizeof(report),
                                  report);

    // Release
    report[0] = 0;
    report[1] = 0;
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA,
                                  RID_CONSUMER,
                                  sizeof(report),
                                  report);
}
