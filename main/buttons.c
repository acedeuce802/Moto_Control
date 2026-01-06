#include "buttons.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "config.h"
#include "timeutil.h"
#include "bt_hid.h"

static const char *TAG = "BUTTONS";

/* ---------------- Raw button read ----------------
   Buttons are assumed active-low with internal pull-ups enabled.
*/
static inline bool btn_raw(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

/* ---------------- GPIO init ---------------- */
void buttons_init(void)
{
    const gpio_num_t pins[] = { PIN_VOL_UP, PIN_VOL_DN, PIN_NEXT, PIN_ANSWER, PIN_HANG, PIN_ASST };

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
    (void)arg;

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
                ESP_LOGW(TAG, "Factory reset combo");
                bt_hid_factory_reset(); // clears bonds + saved host + reboot
                // no return needed; bt_hid_factory_reset() restarts
            } else if (held >= PAIR_HOLD_MS) {
                bt_hid_enter_pairing_mode();
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
                bt_hid_send_consumer(CC_BIT_VOL_UP);
            } else if (held >= VOL_REPEAT_DELAY_MS && (t - b_volu.last_repeat_ms) >= VOL_REPEAT_RATE_MS) {
                b_volu.last_repeat_ms = t;
                bt_hid_send_consumer(CC_BIT_VOL_UP);
            }
        }

        if (b_vold.stable) {
            const int64_t held = t - b_vold.pressed_since_ms;
            if (held == 0) {
                bt_hid_send_consumer(CC_BIT_VOL_DN);
            } else if (held >= VOL_REPEAT_DELAY_MS && (t - b_vold.last_repeat_ms) >= VOL_REPEAT_RATE_MS) {
                b_vold.last_repeat_ms = t;
                bt_hid_send_consumer(CC_BIT_VOL_DN);
            }
        }

        // Edge-triggered actions on press
        if (b_next.stable && (t == b_next.pressed_since_ms)) {
            bt_hid_send_consumer(CC_BIT_NEXT);
        }

        // Answer -> Play/Pause
        if (b_ans.stable && (t == b_ans.pressed_since_ms)) {
            bt_hid_send_consumer(CC_BIT_PLAYPAUSE);
        }

        // Hang -> Pause
        if (b_hang.stable && (t == b_hang.pressed_since_ms)) {
            bt_hid_send_consumer(CC_BIT_PAUSE);
        }

        // Assistant
        if (b_asst.stable && (t == b_asst.pressed_since_ms)) {
            bt_hid_send_consumer(CC_BIT_VOICE);
        }

        delay_ms(SCAN_PERIOD_MS);
    }
}

void buttons_task_start(void)
{
    // Buttons on CPU1 keeps CPU0 freer for BT stack work
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 2, NULL, 1);
}
