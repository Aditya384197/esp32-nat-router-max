idf_component_register(
    SRCS
        "main.c"
        "router_core.c"
        "led_status.c"

    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        lwip
        driver
)
