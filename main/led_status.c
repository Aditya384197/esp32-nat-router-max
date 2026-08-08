#include "led_status.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "LED_STATUS";

/*
 * ============================================================
 * LED CONFIGURATION
 * ============================================================
 *
 * Default ESP32 development-board LED:
 *
 * GPIO 2
 *
 * If your physical board uses another LED GPIO, change only
 * this value.
 */
#define ROUTER_STATUS_LED_GPIO      GPIO_NUM_2

/*
 * Most ESP32 development boards use an active-HIGH LED:
 *
 * HIGH = ON
 * LOW  = OFF
 *
 * If your board has an active-LOW LED, change this to 0.
 */
#define ROUTER_STATUS_LED_ACTIVE_HIGH   1


/* ============================================================
 * INTERNAL STATE
 * ============================================================ */

static bool s_led_initialized = false;
static bool s_led_state = false;


/* ============================================================
 * INTERNAL HELPER
 * ============================================================ */

static void led_write(bool on)
{
    if (!s_led_initialized) {
        return;
    }

#if ROUTER_STATUS_LED_ACTIVE_HIGH
    gpio_set_level(
        ROUTER_STATUS_LED_GPIO,
        on ? 1 : 0
    );
#else
    gpio_set_level(
        ROUTER_STATUS_LED_GPIO,
        on ? 0 : 1
    );
#endif

    s_led_state = on;
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void led_init(void)
{
    if (s_led_initialized) {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROUTER_STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io_conf);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to configure LED GPIO: %s",
            esp_err_to_name(err)
        );
        return;
    }

    s_led_initialized = true;

    /*
     * Start with LED OFF.
     */
    led_write(false);

    ESP_LOGI(
        TAG,
        "Status LED initialized on GPIO %d",
        ROUTER_STATUS_LED_GPIO
    );
}


/* ============================================================
 * LED OFF
 * ============================================================ */

void led_off(void)
{
    led_write(false);
}


/* ============================================================
 * LED ON
 * ============================================================ */

void led_on(void)
{
    led_write(true);
}


/* ============================================================
 * LED TOGGLE
 * ============================================================ */

void led_toggle(void)
{
    if (!s_led_initialized) {
        return;
    }

    led_write(!s_led_state);
}


/* ============================================================
 * CONNECTION STATUS
 * ============================================================ */

void led_set_connected(int connected)
{
    if (!s_led_initialized) {
        return;
    }

    if (connected) {
        led_on();
    } else {
        led_off();
    }
}
