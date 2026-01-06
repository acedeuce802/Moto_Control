#include "led.h"

#include "esp_err.h"
#include "driver/gpio.h"

#include "config.h"
#include "timeutil.h"

static volatile led_state_t s_led_state = LED_BOOTING;

static inline void led_on(void)  { gpio_set_level(PIN_LED, 0); } // active-low sink
static inline void led_off(void) { gpio_set_level(PIN_LED, 1); }

void led_set_state(led_state_t st)
{
    s_led_state = st;
}

static void led_task(void *arg)
{
    // Boot: two quick blinks
    for (int i = 0; i < 2; i++) {
        led_on();  delay_ms(80);
        led_off(); delay_ms(120);
    }

    while (1) {
        switch (s_led_state) {
            case LED_FACTORY_RESET:
                for (int i = 0; i < 3; i++) {
                    led_on();  delay_ms(80);
                    led_off(); delay_ms(80);
                }
                delay_ms(250);
                break;

            case LED_PAIRING:
                led_on();  delay_ms(120);
                led_off(); delay_ms(120);
                break;

            case LED_CONNECTABLE_WAIT:
                led_on();  delay_ms(100);
                led_off(); delay_ms(1900);
                break;

            case LED_CONNECTED:
                led_on();
                delay_ms(500);
                break;

            case LED_BOOTING:
            default:
                led_on();  delay_ms(150);
                led_off(); delay_ms(850);
                break;
        }
    }
}

void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    led_off();
    led_set_state(LED_BOOTING);
}

void led_task_start(void)
{
    // LED task can run on CPU1
    xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, 1, NULL, 1);
}
