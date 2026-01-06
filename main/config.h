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

#pragma once
#include "driver/gpio.h"

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

// Panel LED (active-low sink)
#define PIN_LED      GPIO_NUM_25

/* ---------------- HID (Consumer Control) ---------------- */
#define RID_CONSUMER 0x01

#define CC_BIT_VOL_UP     (1u << 0)
#define CC_BIT_VOL_DN     (1u << 1)
#define CC_BIT_NEXT       (1u << 2)
#define CC_BIT_PLAYPAUSE  (1u << 3)
#define CC_BIT_PAUSE      (1u << 4)
#define CC_BIT_VOICE      (1u << 5)