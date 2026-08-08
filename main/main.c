#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "router_core.h"
#include "led_status.h"

static const char *TAG = "MAIN";


void app_main(void)
{
    /*
     * ========================================================
     * NVS INITIALIZATION
     * ========================================================
     */

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(
            TAG,
            "NVS requires erase/reinitialization"
        );

        ESP_ERROR_CHECK(nvs_flash_erase());

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    /*
     * ========================================================
     * LED
     * ========================================================
     */

    led_init();


    /*
     * ========================================================
     * ROUTER
     * ========================================================
     */

    ESP_LOGI(
        TAG,
        "ESP32 NAT Router starting..."
    );

    ESP_ERROR_CHECK(
        router_core_init()
    );


    /*
     * ========================================================
     * MAIN TASK
     *
     * Router tasks are created by router_core_init().
     * Keep app_main alive.
     * ========================================================
     */

    while (true)
    {
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}
