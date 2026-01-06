// main.c — ESP-IDF (ESP32) Bluetooth Classic HID (Consumer Control) + (optional) BLE-ready
//
// Implements a handlebar controller that pairs to Android (Pixel) as a Classic HID device.
// Buttons (active-low w/ internal pullups):
//   VOL+  (GPIO21)  - volume up (repeat on hold)
//   VOL-  (GPIO22)  - volume down (repeat on hold)
//   NEXT  (GPIO23)  - next track
//   ANSW  (GPIO18)  - play/pause
//   HANG  (GPIO19)  - pause
//   ASST  (GPIO27)  - voice command (best effort)
// Pairing flow (no dedicated pair button):
//   Hold VOL+ + VOL- for 3s  -> enter pairing mode (discoverable/connectable)
//   Hold VOL+ + VOL- for 10s -> factory reset (clear bonds) + reboot
//
// Build target: esp32
// Host stack: Bluedroid
// Controller mode: BTDM (Classic + BLE)
//
// Notes:
// - This uses the Bluetooth Classic HID Device API from esp_hidd_api.h (esp_bt_hid_device_*).
// - BLE GATT is not implemented here (stub only). HID over Classic coexists cleanly with headset audio.

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#include "esp_hidd_api.h"

#include "nvs.h"

static const char *TAG = "MOTO_CTRL";

/* ---------------- Pins (active-low buttons) ---------------- */
#define PIN_VOL_UP   GPIO_NUM_21
#define PIN_VOL_DN   GPIO_NUM_22
#define PIN_NEXT     GPIO_NUM_23
#define PIN_ANSWER   GPIO_NUM_18
#define PIN_HANG     GPIO_NUM_19
#define PIN_ASST     GPIO_NUM_27

/* ---------------- Timing ---------------- */
#define SCAN_PERIOD_MS          5
#define DEBOUNCE_MS            15
#define PAIR_HOLD_MS         3000
#define RESET_HOLD_MS       10000
#define VOL_REPEAT_DELAY_MS    400
#define VOL_REPEAT_RATE_MS     140

/* ---------------- HID (Consumer Control) ---------------- */
#define RID_CONSUMER 0x01

#define CC_BIT_VOL_UP     (1u << 0)
#define CC_BIT_VOL_DN     (1u << 1)
#define CC_BIT_NEXT       (1u << 2)
#define CC_BIT_PLAYPAUSE  (1u << 3)
#define CC_BIT_PAUSE      (1u << 4)
#define CC_BIT_VOICE      (1u << 5)

/* ---------------- NVS ---------------- */
#define NVS_NS "moto"
#define NVS_KEY_HOST "host_bda"   // 6 bytes

/* ---------------- Globals ---------------- */
static bool s_hid_ready    = false;
static bool s_hid_connected = false;
static bool s_pairing_mode = false;

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

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static inline bool btn_raw(gpio_num_t pin) { return gpio_get_level(pin) == 0; } // pressed

/* ---------------- NVS helper ---------------- */
static void nvs_save_host_bda(const esp_bd_addr_t bda)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_HOST, bda, ESP_BD_ADDR_LEN);
    nvs_commit(h);
    nvs_close(h);
}

static bool nvs_load_host_bda(esp_bd_addr_t out_bda)
{
    nvs_handle_t h;
    size_t len = ESP_BD_ADDR_LEN;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    esp_err_t err = nvs_get_blob(h, NVS_KEY_HOST, out_bda, &len);
    nvs_close(h);
    return (err == ESP_OK && len == ESP_BD_ADDR_LEN);
}

/* ---------------- Discoverability helper ---------------- */
static void bt_set_pairing_mode(bool enable)
{
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                             enable ? ESP_BT_GENERAL_DISCOVERABLE : ESP_BT_NON_DISCOVERABLE);
}

/* ---------------- Clear bonds (factory reset) ---------------- */
static void clear_all_bonds(void)
{
    int dev_num = esp_bt_gap_get_bond_device_num();
    if (dev_num <= 0) {
        ESP_LOGW(TAG, "No bonded devices to clear.");
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

/* ---------------- Pairing mode ---------------- */
static void enter_pairing_mode(void)
{
    if (s_pairing_mode) return;

    s_pairing_mode = true;
    ESP_LOGI(TAG, "Entering pairing mode (discoverable/connectable)...");
    bt_set_pairing_mode(true);
}

/* ---------------- HID send: press then release ---------------- */
static void hid_send_consumer_bits(uint16_t bits)
{
    if (!s_hid_ready || !s_hid_connected) return;

    uint8_t report[2] = {
        (uint8_t)(bits & 0xFF),
        (uint8_t)((bits >> 8) & 0xFF)
    };

    // Send input report (interrupt channel)
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, RID_CONSUMER, sizeof(report), report);

    // Release
    report[0] = 0;
    report[1] = 0;
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, RID_CONSUMER, sizeof(report), report);
}

/* ---------------- GAP callback (pairing/security) ---------------- */
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
            // "Just Works" confirmation request
            ESP_LOGI(TAG, "SSP confirm request, numeric=%" PRIu32, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(TAG, "SSP key notif: %" PRIu32, param->key_notif.passkey);
            break;

        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(TAG, "SSP key request");
            break;

        default:
            break;
    }
}

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

static void nvs_erase_host_bda(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY_HOST);
    nvs_commit(h);
    nvs_close(h);
}

/* ---------------- HID callback ---------------- */
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

        case ESP_HIDD_OPEN_EVT:
            ESP_LOGI(TAG, "HID connected");
            s_hid_connected = true;

            esp_bd_addr_t bda;
            if (get_first_bonded_host(bda)) {
                nvs_save_host_bda(bda);
                ESP_LOGI(TAG, "Saved bonded host as last host.");
            } else {
                ESP_LOGW(TAG, "No bonded host found to save.");
            }

            // Exit pairing mode: remain connectable, just stop being discoverable.
            s_pairing_mode = false;
            bt_set_pairing_mode(false);
            break;


        case ESP_HIDD_CLOSE_EVT:
            ESP_LOGI(TAG, "HID disconnected");
            s_hid_connected = false;

            if (s_pairing_mode) {
                // User requested pairing: keep discoverable.
                bt_set_pairing_mode(true);
            } else {
                // Normal behavior: allow phone to reconnect after power cycles.
                bt_set_pairing_mode(false);   // connectable, non-discoverable
            }
            break;


        default:
            break;
    }
}

static void delay_ms(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    if (t < 1) t = 1;
    vTaskDelay(t);
}

static void hid_autoconnect_task(void *arg)
{
    // Give Android a moment after accessory power-up
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

    // Retry a few times with backoff
    for (int i = 0; i < 5 && !s_hid_connected; i++) {
        esp_err_t err = esp_bt_hid_device_connect(bda);
        ESP_LOGI(TAG, "connect attempt %d -> %s", i + 1, esp_err_to_name(err));

        if (err == ESP_OK) {
            // Connection attempt started; OPEN event should follow.
            delay_ms(1500);
        } else if (err == ESP_ERR_INVALID_STATE) {
            // Stack not ready yet; back off more.
            delay_ms(2500);
        } else {
            delay_ms(1500 + i * 500);
        }
            }

    vTaskDelete(NULL);
}

/* ---------------- GPIO init ---------------- */
static void buttons_init(void)
{
    const gpio_num_t pins[] = {PIN_VOL_UP, PIN_VOL_DN, PIN_NEXT, PIN_ANSWER, PIN_HANG, PIN_ASST};

    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = 0
    };

    for (int i = 0; i < (int)(sizeof(pins) / sizeof(pins[0])); i++) {
        io.pin_bit_mask |= (1ULL << pins[i]);
    }
    ESP_ERROR_CHECK(gpio_config(&io));
}

/* ---------------- Button scan task ---------------- */
typedef struct {
    bool stable;            // debounced stable level (pressed=true)
    bool last_raw;          // last sampled raw level
    int64_t last_change_ms; // last time raw changed
    int64_t pressed_since_ms;
    int64_t last_repeat_ms;
} btn_state_t;

static void debounce_update(btn_state_t *s, bool raw, int64_t t_ms)
{
    if (raw != s->last_raw) {
        s->last_raw = raw;
        s->last_change_ms = t_ms;
    }

    if ((t_ms - s->last_change_ms) >= DEBOUNCE_MS) {
        if (s->stable != s->last_raw) {
            s->stable = s->last_raw;
            if (s->stable) {
                s->pressed_since_ms = t_ms;
                s->last_repeat_ms = t_ms;
            }
        }
    }
}

static void button_task(void *arg)
{
    btn_state_t b_volu = {0}, b_vold = {0}, b_next = {0}, b_ans = {0}, b_hang = {0}, b_asst = {0};

    int64_t combo_press_start = -1;

    while (1) {
        const int64_t t = now_ms();

        const bool r_volu = btn_raw(PIN_VOL_UP);
        const bool r_vold = btn_raw(PIN_VOL_DN);

        // Pairing/reset combo (raw hold)
        if (r_volu && r_vold) {
            if (combo_press_start < 0) combo_press_start = t;

            const int64_t held = t - combo_press_start;
            if (held >= RESET_HOLD_MS) {
                ESP_LOGW(TAG, "Factory reset combo: clearing bonds then reboot.");
                nvs_erase_host_bda();
                clear_all_bonds();
                delay_ms(200);
                esp_restart();
            } else if (held >= PAIR_HOLD_MS) {
                enter_pairing_mode();
            }
        } else {
            combo_press_start = -1;
        }

        debounce_update(&b_volu, r_volu, t);
        debounce_update(&b_vold, r_vold, t);
        debounce_update(&b_next, btn_raw(PIN_NEXT), t);
        debounce_update(&b_ans,  btn_raw(PIN_ANSWER), t);
        debounce_update(&b_hang, btn_raw(PIN_HANG), t);
        debounce_update(&b_asst, btn_raw(PIN_ASST), t);

        // Volume repeat
        if (b_volu.stable) {
            const int64_t held = t - b_volu.pressed_since_ms;
            if (held == 0) {
                hid_send_consumer_bits(CC_BIT_VOL_UP);
            } else if (held >= VOL_REPEAT_DELAY_MS && (t - b_volu.last_repeat_ms) >= VOL_REPEAT_RATE_MS) {
                b_volu.last_repeat_ms = t;
                hid_send_consumer_bits(CC_BIT_VOL_UP);
            }
        }
        if (b_vold.stable) {
            const int64_t held = t - b_vold.pressed_since_ms;
            if (held == 0) {
                hid_send_consumer_bits(CC_BIT_VOL_DN);
            } else if (held >= VOL_REPEAT_DELAY_MS && (t - b_vold.last_repeat_ms) >= VOL_REPEAT_RATE_MS) {
                b_vold.last_repeat_ms = t;
                hid_send_consumer_bits(CC_BIT_VOL_DN);
            }
        }

        // Edge-triggered actions on press
        if (b_next.stable && (t == b_next.pressed_since_ms)) {
            hid_send_consumer_bits(CC_BIT_NEXT);
        }

        // Answer -> Play/Pause (your requested behavior)
        if (b_ans.stable && (t == b_ans.pressed_since_ms)) {
            hid_send_consumer_bits(CC_BIT_PLAYPAUSE);
        }

        // Hang -> Pause (your requested behavior)
        if (b_hang.stable && (t == b_hang.pressed_since_ms)) {
            hid_send_consumer_bits(CC_BIT_PAUSE);
        }

        // Assistant (best-effort)
        if (b_asst.stable && (t == b_asst.pressed_since_ms)) {
            hid_send_consumer_bits(CC_BIT_VOICE);
        }

        TickType_t d = pdMS_TO_TICKS(SCAN_PERIOD_MS);
        if (d < 1) d = 1;
        vTaskDelay(d);
    }
}

/* ---------------- BLE stub ----------------
   Dual-mode (BTDM) is enabled, so you can add BLE GATT later without changing the controller mode.
   This function is intentionally empty for now.
*/
static void ble_gatt_init_stub(void)
{
    // Add BLE GATT services later (status/config/battery) once HID is stable.
}

static void bt_init_dual_mode_hid(void)
{
    static bool s_bt_inited = false;
    if (s_bt_inited) {
        ESP_LOGW(TAG, "BT already initialized; skipping bt_init_dual_mode_hid()");
        return;
    }
    s_bt_inited = true;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM)); // Classic + BLE

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // GAP callback (handles SSP confirm, auth complete, etc.)
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));

    // Use the non-deprecated call
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name("MotoControls"));

    // SSP "Just Works"
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap)));

    // HID Device init (Classic HID Device API)
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

    // IMPORTANT for ESP-IDF v5.5.2: do not pass NULL QoS pointers
    static esp_hidd_qos_param_t s_qos_in  = { 0 };
    static esp_hidd_qos_param_t s_qos_out = { 0 };

    ESP_ERROR_CHECK(esp_bt_hid_device_register_app(&app_param, &s_qos_in, &s_qos_out));

    // After HID is registered, set scan mode based on whether we already have bonds.
    int bonds = esp_bt_gap_get_bond_device_num();
    ESP_LOGI(TAG, "Bonded devices: %d", bonds);

    if (bonds == 0) {
        ESP_LOGI(TAG, "No bonds; enabling discoverable/connectable for initial pairing.");
        enter_pairing_mode();          // connectable + discoverable
    } else {
        bt_set_pairing_mode(false);    // connectable + NOT discoverable (reconnect works)
        xTaskCreatePinnedToCore(hid_autoconnect_task, "hid_autoconnect", 4096, NULL, 3, NULL, 0);
    }

}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    buttons_init();
    bt_init_dual_mode_hid();
    ble_gatt_init_stub();

    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "Ready. Hold VOL+ + VOL- for 3s to pair, 10s to factory reset.");
}
