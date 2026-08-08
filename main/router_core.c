/*
 * ============================================================
 * ESP32 NAT ROUTER CORE
 * ESP-IDF v5.1.2 / ESP32 CLASSIC
 *
 * Features
 * ------------------------------------------------------------
 * - AP + STA simultaneous operation
 * - NVS persistent configuration
 * - DHCP server on AP
 * - DHCP client on STA
 * - IPv4 forwarding
 * - IPv4 NAPT
 * - Automatic STA reconnect
 * - Clean line-based serial console
 * - Runtime statistics
 * - AP / STA reset commands
 *
 * Console
 * ------------------------------------------------------------
 * AP
 * STA
 * AP RESET
 * STA RESET
 * RESET
 * STATUS
 * RECONNECT
 * HELP
 *
 * AP / STA configuration uses:
 *
 * AP <ssid> <password>
 * STA <ssid> <password>
 *
 * Example:
 *
 * AP MyRouter 12345678
 * STA HomeWiFi MyPassword
 *
 * ============================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_system.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#include "router_core.h"
#include "router_config.h"
#include "led_status.h"


/* ============================================================
 * TAGS
 * ============================================================ */

static const char *TAG = "ROUTER_CORE";


/* ============================================================
 * EVENT BITS
 * ============================================================ */

#define ROUTER_EVENT_STA_CONNECTED   BIT0
#define ROUTER_EVENT_STA_GOT_IP      BIT1
#define ROUTER_EVENT_WIFI_STARTED    BIT2


/* ============================================================
 * CONSOLE
 * ============================================================ */

#define CONSOLE_BUFFER_SIZE          160
#define CONSOLE_TASK_STACK           4096
#define CONSOLE_TASK_PRIORITY        3

#define STATS_TASK_STACK             4096
#define STATS_TASK_PRIORITY          2

#define RECONNECT_TASK_STACK         4096
#define RECONNECT_TASK_PRIORITY      5


/* ============================================================
 * GLOBAL STATE
 * ============================================================ */

router_config_t g_router = {
    .sta_connected  = false,
    .sta_rssi       = -127,
    .sta_channel    = 0,
    .ap_clients     = 0,
    .nat_enabled    = false,
    .uptime_sec     = 0,
    .reconnect_count = 0,
    .napt_failures  = 0
};


router_saved_config_t g_saved_config = {
    .ap_ssid = AP_DEFAULT_SSID,
    .ap_password = AP_DEFAULT_PASSWORD,
    .sta_ssid = "",
    .sta_password = "",
    .ap_configured = false,
    .sta_configured = false
};


/* ============================================================
 * INTERNAL OBJECTS
 * ============================================================ */

static esp_netif_t *s_ap_netif = NULL;
static esp_netif_t *s_sta_netif = NULL;

static EventGroupHandle_t s_router_events = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;

static TaskHandle_t s_console_task = NULL;
static TaskHandle_t s_reconnect_task = NULL;
static TaskHandle_t s_stats_task = NULL;

static bool s_initialized = false;
static bool s_wifi_started = false;

static uint32_t s_last_reconnect_ms = 0;
static uint32_t s_reconnect_delay_ms = STA_RECONNECT_INITIAL_MS;


/* ============================================================
 * FORWARD DECLARATIONS
 * ============================================================ */

static esp_err_t load_configuration(void);
static esp_err_t save_configuration(void);

static esp_err_t erase_ap_configuration(void);
static esp_err_t erase_sta_configuration(void);
static esp_err_t erase_all_configuration(void);

static esp_err_t configure_ap(void);
static esp_err_t configure_sta(void);

static esp_err_t start_wifi(void);

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
);

static void reconnect_task(void *arg);
static void console_task(void *arg);

static void print_status(void);
static void print_help(void);

static void process_command(char *line);

static bool valid_ssid(const char *ssid);
static bool valid_password(const char *password);

static void trim_line(char *line);
static void update_sta_statistics(void);

static esp_err_t enable_napt(void);
static void disable_napt(void);


/* ============================================================
 * SMALL UTILITY FUNCTIONS
 * ============================================================ */

static bool valid_ssid(const char *ssid)
{
    if (ssid == NULL) {
        return false;
    }

    size_t len = strlen(ssid);

    return (len >= 1U &&
            len <= ROUTER_SSID_MAX_LEN);
}


static bool valid_password(const char *password)
{
    if (password == NULL) {
        return false;
    }

    size_t len = strlen(password);

    /*
     * Empty password is allowed for the default/open AP.
     *
     * For WPA2, caller must provide 8..63 characters.
     */
    if (len == 0U) {
        return true;
    }

    return (len >= 8U &&
            len <= ROUTER_PASSWORD_MAX_LEN);
}


static void trim_line(char *line)
{
    if (line == NULL) {
        return;
    }

    size_t len = strlen(line);

    while (len > 0U &&
           (line[len - 1U] == '\r' ||
            line[len - 1U] == '\n' ||
            line[len - 1U] == ' ' ||
            line[len - 1U] == '\t')) {
        line[len - 1U] = '\0';
        len--;
    }

    char *start = line;

    while (*start == ' ' || *start == '\t') {
        start++;
    }

    if (start != line) {
        memmove(line, start, strlen(start) + 1U);
    }
}


/* ============================================================
 * NVS LOAD
 * ============================================================ */

static esp_err_t load_configuration(void)
{
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(
        ROUTER_NVS_NAMESPACE,
        NVS_READONLY,
        &nvs
    );

    /*
     * No namespace yet is a normal first-boot condition.
     */
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(
            g_saved_config.ap_ssid,
            AP_DEFAULT_SSID
        );

        strcpy(
            g_saved_config.ap_password,
            AP_DEFAULT_PASSWORD
        );

        g_saved_config.sta_ssid[0] = '\0';
        g_saved_config.sta_password[0] = '\0';

        g_saved_config.ap_configured = false;
        g_saved_config.sta_configured = false;

        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }


    /* --------------------------------------------------------
     * AP SSID
     * -------------------------------------------------------- */

    size_t len = sizeof(g_saved_config.ap_ssid);

    err = nvs_get_str(
        nvs,
        NVS_KEY_AP_SSID,
        g_saved_config.ap_ssid,
        &len
    );

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }


    /* --------------------------------------------------------
     * AP PASSWORD
     * -------------------------------------------------------- */

    len = sizeof(g_saved_config.ap_password);

    err = nvs_get_str(
        nvs,
        NVS_KEY_AP_PASS,
        g_saved_config.ap_password,
        &len
    );

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }


    /* --------------------------------------------------------
     * STA SSID
     * -------------------------------------------------------- */

    len = sizeof(g_saved_config.sta_ssid);

    err = nvs_get_str(
        nvs,
        NVS_KEY_STA_SSID,
        g_saved_config.sta_ssid,
        &len
    );

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }


    /* --------------------------------------------------------
     * STA PASSWORD
     * -------------------------------------------------------- */

    len = sizeof(g_saved_config.sta_password);

    err = nvs_get_str(
        nvs,
        NVS_KEY_STA_PASS,
        g_saved_config.sta_password,
        &len
    );

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs);
        return err;
    }

    nvs_close(nvs);


    /*
     * Validate loaded values.
     *
     * Invalid/corrupted NVS must never reach esp_wifi.
     */
    if (!valid_ssid(g_saved_config.ap_ssid)) {
        strcpy(
            g_saved_config.ap_ssid,
            AP_DEFAULT_SSID
        );

        g_saved_config.ap_password[0] = '\0';
        g_saved_config.ap_configured = false;
    } else {
        g_saved_config.ap_configured = true;
    }

    if (!valid_password(g_saved_config.ap_password)) {
        g_saved_config.ap_password[0] = '\0';
        g_saved_config.ap_configured = false;
    }

    if (g_saved_config.sta_ssid[0] != '\0') {
        if (!valid_ssid(g_saved_config.sta_ssid)) {
            g_saved_config.sta_ssid[0] = '\0';
            g_saved_config.sta_password[0] = '\0';
            g_saved_config.sta_configured = false;
        } else {
            g_saved_config.sta_configured = true;
        }
    } else {
        g_saved_config.sta_configured = false;
    }

    return ESP_OK;
}


/* ============================================================
 * NVS SAVE
 * ============================================================ */

static esp_err_t save_configuration(void)
{
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(
        ROUTER_NVS_NAMESPACE,
        NVS_READWRITE,
        &nvs
    );

    if (err != ESP_OK) {
        return err;
    }


    err = nvs_set_str(
        nvs,
        NVS_KEY_AP_SSID,
        g_saved_config.ap_ssid
    );

    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }


    err = nvs_set_str(
        nvs,
        NVS_KEY_AP_PASS,
        g_saved_config.ap_password
    );

    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }


    err = nvs_set_str(
        nvs,
        NVS_KEY_STA_SSID,
        g_saved_config.sta_ssid
    );

    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }


    err = nvs_set_str(
        nvs,
        NVS_KEY_STA_PASS,
        g_saved_config.sta_password
    );

    if (err != ESP_OK) {
        nvs_close(nvs);
        return err;
    }


    err = nvs_commit(nvs);

    nvs_close(nvs);

    return err;
}


/* ============================================================
 * NVS RESET FUNCTIONS
 * ============================================================ */

static esp_err_t erase_ap_configuration(void)
{
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(
        ROUTER_NVS_NAMESPACE,
        NVS_READWRITE,
        &nvs
    );

    if (err != ESP_OK) {
        return err;
    }

    nvs_erase_key(nvs, NVS_KEY_AP_SSID);
    nvs_erase_key(nvs, NVS_KEY_AP_PASS);

    err = nvs_commit(nvs);

    nvs_close(nvs);

    if (err == ESP_OK) {
        strcpy(
            g_saved_config.ap_ssid,
            AP_DEFAULT_SSID
        );

        g_saved_config.ap_password[0] = '\0';
        g_saved_config.ap_configured = false;
    }

    return err;
}


static esp_err_t erase_sta_configuration(void)
{
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(
        ROUTER_NVS_NAMESPACE,
        NVS_READWRITE,
        &nvs
    );

    if (err != ESP_OK) {
        return err;
    }

    nvs_erase_key(nvs, NVS_KEY_STA_SSID);
    nvs_erase_key(nvs, NVS_KEY_STA_PASS);

    err = nvs_commit(nvs);

    nvs_close(nvs);

    if (err == ESP_OK) {
        g_saved_config.sta_ssid[0] = '\0';
        g_saved_config.sta_password[0] = '\0';
        g_saved_config.sta_configured = false;
    }

    return err;
}


static esp_err_t erase_all_configuration(void)
{
    nvs_handle_t nvs;

    esp_err_t err = nvs_open(
        ROUTER_NVS_NAMESPACE,
        NVS_READWRITE,
        &nvs
    );

    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_all(nvs);

    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);

    if (err == ESP_OK) {

        strcpy(
            g_saved_config.ap_ssid,
            AP_DEFAULT_SSID
        );

        g_saved_config.ap_password[0] = '\0';

        g_saved_config.sta_ssid[0] = '\0';
        g_saved_config.sta_password[0] = '\0';

        g_saved_config.ap_configured = false;
        g_saved_config.sta_configured = false;
    }

    return err;
}


/* ============================================================
 * AP CONFIGURATION
 * ============================================================ */

static esp_err_t configure_ap(void)
{
    if (s_ap_netif == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    wifi_config_t ap_config;

    memset(
        &ap_config,
        0,
        sizeof(ap_config)
    );


    /* --------------------------------------------------------
     * SSID
     * -------------------------------------------------------- */

    strlcpy(
        (char *)ap_config.ap.ssid,
        g_saved_config.ap_ssid,
        sizeof(ap_config.ap.ssid)
    );

    ap_config.ap.ssid_len =
        strlen(g_saved_config.ap_ssid);


    /* --------------------------------------------------------
     * Password / Security
     * -------------------------------------------------------- */

    if (strlen(g_saved_config.ap_password) >= 8U) {

        strlcpy(
            (char *)ap_config.ap.password,
            g_saved_config.ap_password,
            sizeof(ap_config.ap.password)
        );

        ap_config.ap.authmode =
            WIFI_AUTH_WPA2_PSK;

    } else {

        ap_config.ap.password[0] = '\0';

        ap_config.ap.authmode =
            WIFI_AUTH_OPEN;
    }


    /* --------------------------------------------------------
     * AP channel
     * -------------------------------------------------------- */

    ap_config.ap.channel =
        AP_INITIAL_CHANNEL;


    /* --------------------------------------------------------
     * AP clients
     * -------------------------------------------------------- */

    ap_config.ap.max_connection =
        AP_MAX_CONN;


    /* --------------------------------------------------------
     * PMF
     * -------------------------------------------------------- */

    ap_config.ap.pmf_cfg.required = false;
    ap_config.ap.pmf_cfg.capable = true;


    return esp_wifi_set_config(
        WIFI_IF_AP,
        &ap_config
    );
}


/* ============================================================
 * STA CONFIGURATION
 * ============================================================ */

static esp_err_t configure_sta(void)
{
    wifi_config_t sta_config;

    memset(
        &sta_config,
        0,
        sizeof(sta_config)
    );


    if (!g_saved_config.sta_configured ||
        !valid_ssid(g_saved_config.sta_ssid)) {

        /*
         * No STA configuration.
         *
         * Keep interface configured but do not attempt
         * connection.
         */
        sta_config.sta.ssid[0] = '\0';
        sta_config.sta.password[0] = '\0';

        return esp_wifi_set_config(
            WIFI_IF_STA,
            &sta_config
        );
    }


    strlcpy(
        (char *)sta_config.sta.ssid,
        g_saved_config.sta_ssid,
        sizeof(sta_config.sta.ssid)
    );


    strlcpy(
        (char *)sta_config.sta.password,
        g_saved_config.sta_password,
        sizeof(sta_config.sta.password)
    );


    /*
     * Do not force a channel here.
     *
     * In AP+STA mode the STA association determines the
     * radio channel and the AP follows it.
     */
    sta_config.sta.channel = 0;


    sta_config.sta.scan_method =
        WIFI_ALL_CHANNEL_SCAN;

    sta_config.sta.sort_method =
        WIFI_CONNECT_AP_BY_SIGNAL;


    sta_config.sta.threshold.authmode =
        WIFI_AUTH_OPEN;


    /*
     * PMF remains optional for compatibility with a broad
     * range of upstream routers.
     */
    sta_config.sta.pmf_cfg.capable = true;
    sta_config.sta.pmf_cfg.required = false;


    return esp_wifi_set_config(
        WIFI_IF_STA,
        &sta_config
    );
}


/* ============================================================
 * NAPT
 * ============================================================ */

static void disable_napt(void)
{
    if (s_ap_netif == NULL) {
        atomic_store(
            &g_router.nat_enabled,
            false
        );

        return;
    }


    esp_err_t err =
        esp_netif_napt_disable(s_ap_netif);

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGW(
            TAG,
            "NAPT disable: %s",
            esp_err_to_name(err)
        );
    }


    atomic_store(
        &g_router.nat_enabled,
        false
    );
}


static esp_err_t enable_napt(void)
{
    if (s_ap_netif == NULL ||
        s_sta_netif == NULL) {

        return ESP_ERR_INVALID_STATE;
    }


    /*
     * NAPT is required on the AP-side netif.
     *
     * The STA netif provides the upstream route.
     */
    esp_err_t err =
        esp_netif_napt_enable(s_ap_netif);

    if (err != ESP_OK) {

        atomic_store(
            &g_router.nat_enabled,
            false
        );

        atomic_fetch_add(
            &g_router.napt_failures,
            1
        );

        ESP_LOGE(
            TAG,
            "NAPT enable failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    atomic_store(
        &g_router.nat_enabled,
        true
    );

    ESP_LOGI(
        TAG,
        "IPv4 NAPT enabled"
    );

    return ESP_OK;
}


/* ============================================================
 * WIFI START
 * ============================================================ */

static esp_err_t start_wifi(void)
{
    if (s_wifi_started) {
        return ESP_OK;
    }


    wifi_init_config_t wifi_init_cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    esp_err_t err =
        esp_wifi_init(&wifi_init_cfg);

    if (err != ESP_OK) {
        return err;
    }


    /*
     * Router mode requires AP + STA simultaneously.
     */
    err = esp_wifi_set_mode(
        WIFI_MODE_APSTA
    );

    if (err != ESP_OK) {
        return err;
    }


    /*
     * Disable modem sleep.
     *
     * This favors latency/throughput over power saving.
     */
    err = esp_wifi_set_ps(
        ROUTER_WIFI_PS
    );

    if (err != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Could not disable WiFi power save: %s",
            esp_err_to_name(err)
        );
    }


    /*
     * Configure AP.
     */
    err = configure_ap();

    if (err != ESP_OK) {
        return err;
    }


    /*
     * Configure STA.
     */
    err = configure_sta();

    if (err != ESP_OK) {
        return err;
    }


    /*
     * TX power.
     *
     * Hardware/regulatory limits still apply.
     */
    err = esp_wifi_set_max_tx_power(
        ROUTER_MAX_TX_POWER
    );

    if (err != ESP_OK) {
        ESP_LOGW(
            TAG,
            "TX power setting failed: %s",
            esp_err_to_name(err)
        );
    }


    err = esp_wifi_start();

    if (err != ESP_OK) {
        return err;
    }


    s_wifi_started = true;


    xEventGroupSetBits(
        s_router_events,
        ROUTER_EVENT_WIFI_STARTED
    );


    ESP_LOGI(
        TAG,
        "WiFi AP+STA started"
    );


    /*
     * Connect only when STA configuration exists.
     */
    if (g_saved_config.sta_configured) {

        err = esp_wifi_connect();

        if (err != ESP_OK) {
            ESP_LOGW(
                TAG,
                "Initial STA connect failed: %s",
                esp_err_to_name(err)
            );
        }
    }


    return ESP_OK;
}


/* ============================================================
 * WIFI EVENT HANDLER
 * ============================================================ */

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    (void)arg;


    /* ========================================================
     * WIFI EVENTS
     * ======================================================== */

    if (event_base == WIFI_EVENT) {

        switch (event_id) {

            case WIFI_EVENT_STA_START:

                if (g_saved_config.sta_configured) {

                    esp_wifi_connect();
                }

                break;


            case WIFI_EVENT_STA_CONNECTED:
            {
                wifi_event_sta_connected_t *event =
                    (wifi_event_sta_connected_t *)event_data;

                atomic_store(
                    &g_router.sta_channel,
                    event->channel
                );

                atomic_store(
                    &g_router.sta_connected,
                    true
                );

                xEventGroupSetBits(
                    s_router_events,
                    ROUTER_EVENT_STA_CONNECTED
                );

                s_reconnect_delay_ms =
                    STA_RECONNECT_INITIAL_MS;

                ESP_LOGI(
                    TAG,
                    "STA connected: channel=%d",
                    event->channel
                );

                break;
            }


            case WIFI_EVENT_STA_DISCONNECTED:
            {
                wifi_event_sta_disconnected_t *event =
                    (wifi_event_sta_disconnected_t *)event_data;

                atomic_store(
                    &g_router.sta_connected,
                    false
                );

                atomic_store(
                    &g_router.sta_channel,
                    0
                );

                atomic_store(
                    &g_router.sta_rssi,
                    -127
                );

                atomic_store(
                    &g_router.nat_enabled,
                    false
                );

                xEventGroupClearBits(
                    s_router_events,
                    ROUTER_EVENT_STA_CONNECTED |
                    ROUTER_EVENT_STA_GOT_IP
                );


                /*
                 * NAPT must not remain active when the upstream
                 * route is gone.
                 */
                disable_napt();

                led_set_connected(0);


                ESP_LOGW(
                    TAG,
                    "STA disconnected, reason=%d",
                    event->reason
                );

                break;
            }


            case WIFI_EVENT_AP_STACONNECTED:
            {
                atomic_fetch_add(
                    &g_router.ap_clients,
                    1
                );

                break;
            }


            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                int old_clients =
                    atomic_load(
                        &g_router.ap_clients
                    );

                if (old_clients > 0) {

                    atomic_fetch_sub(
                        &g_router.ap_clients,
                        1
                    );
                }

                break;
            }


            default:
                break;
        }

        return;
    }


    /* ========================================================
     * IP EVENTS
     * ======================================================== */

    if (event_base == IP_EVENT) {

        if (event_id == IP_EVENT_STA_GOT_IP) {

            ip_event_got_ip_t *event =
                (ip_event_got_ip_t *)event_data;


            xEventGroupSetBits(
                s_router_events,
                ROUTER_EVENT_STA_GOT_IP
            );


            atomic_store(
                &g_router.sta_connected,
                true
            );


            /*
             * Upstream IP is now available.
             *
             * Only now enable NAPT.
             */
            esp_err_t err =
                enable_napt();

            if (err == ESP_OK) {

                led_set_connected(1);

            } else {

                led_set_connected(0);
            }


            ESP_LOGI(
                TAG,
                "STA IP: " IPSTR,
                IP2STR(&event->ip_info.ip)
            );

            ESP_LOGI(
                TAG,
                "Gateway: " IPSTR,
                IP2STR(&event->ip_info.gw)
            );

            ESP_LOGI(
                TAG,
                "Netmask: " IPSTR,
                IP2STR(&event->ip_info.netmask)
            );

            return;
        }
    }
}


/* ============================================================
 * UPDATE STA RSSI / CHANNEL
 * ============================================================ */

static void update_sta_statistics(void)
{
    if (!s_wifi_started) {
        return;
    }


    wifi_ap_record_t ap_info;

    memset(
        &ap_info,
        0,
        sizeof(ap_info)
    );


    esp_err_t err =
        esp_wifi_sta_get_ap_info(&ap_info);

    if (err == ESP_OK) {

        atomic_store(
            &g_router.sta_rssi,
            ap_info.rssi
        );

        atomic_store(
            &g_router.sta_channel,
            ap_info.primary
        );

    } else {

        atomic_store(
            &g_router.sta_rssi,
            -127
        );
    }
}


/* ============================================================
 * STATUS
 * ============================================================ */

static void print_status(void)
{
    bool sta_connected =
        atomic_load(
            &g_router.sta_connected
        );

    bool nat_enabled =
        atomic_load(
            &g_router.nat_enabled
        );

    int rssi =
        atomic_load(
            &g_router.sta_rssi
        );

    int channel =
        atomic_load(
            &g_router.sta_channel
        );

    int clients =
        atomic_load(
            &g_router.ap_clients
        );

    unsigned uptime =
        atomic_load(
            &g_router.uptime_sec
        );

    unsigned reconnects =
        atomic_load(
            &g_router.reconnect_count
        );

    unsigned napt_failures =
        atomic_load(
            &g_router.napt_failures
        );


    printf("\r\n");
    printf("=========== ROUTER STATUS ===========\r\n");

    printf(
        "AP configured : %s\r\n",
        g_saved_config.ap_configured
            ? "YES"
            : "NO"
    );

    printf(
        "AP SSID       : %s\r\n",
        g_saved_config.ap_ssid
    );

    printf(
        "STA configured: %s\r\n",
        g_saved_config.sta_configured
            ? "YES"
            : "NO"
    );

    printf(
        "STA connected : %s\r\n",
        sta_connected
            ? "YES"
            : "NO"
    );

    printf(
        "STA RSSI      : %d dBm\r\n",
        rssi
    );

    printf(
        "STA channel   : %d\r\n",
        channel
    );

    printf(
        "AP clients    : %d\r\n",
        clients
    );

    printf(
        "NAPT          : %s\r\n",
        nat_enabled
            ? "ON"
            : "OFF"
    );

    printf(
        "Reconnects    : %u\r\n",
        reconnects
    );

    printf(
        "NAPT failures : %u\r\n",
        napt_failures
    );

    printf(
        "Uptime        : %u sec\r\n",
        uptime
    );

    printf(
        "=====================================\r\n"
    );

    printf("\r\n");
}


/* ============================================================
 * HELP
 * ============================================================ */

static void print_help(void)
{
    printf("\r\n");
    printf("=========== ROUTER CONSOLE ===========\r\n");

    printf("STATUS\r\n");
    printf("HELP\r\n");
    printf("RECONNECT\r\n");

    printf("\r\n");

    printf(
        "AP <ssid> <password>\r\n"
    );

    printf(
        "STA <ssid> <password>\r\n"
    );

    printf("\r\n");

    printf("AP RESET\r\n");
    printf("STA RESET\r\n");
    printf("RESET\r\n");

    printf("\r\n");

    printf(
        "Examples:\r\n"
    );

    printf(
        "  AP MyRouter 12345678\r\n"
    );

    printf(
        "  STA HomeWiFi MyPassword\r\n"
    );

    printf("\r\n");

    printf(
        "SSID : 1..32 bytes\r\n"
    );

    printf(
        "WPA2 password : 8..63 characters\r\n"
    );

    printf(
        "======================================\r\n"
    );

    printf("\r\n");
}


/* ============================================================
 * RESTART WIFI CONFIGURATION
 * ============================================================ */

static void restart_wifi_after_config_change(void)
{
    if (!s_wifi_started) {
        return;
    }


    /*
     * Disconnect first so stale STA state does not survive
     * configuration changes.
     */
    esp_wifi_disconnect();


    disable_napt();


    atomic_store(
        &g_router.sta_connected,
        false
    );

    atomic_store(
        &g_router.sta_rssi,
        -127
    );

    atomic_store(
        &g_router.sta_channel,
        0
    );


    /*
     * Reapply AP + STA configuration.
     */
    if (configure_ap() != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to reconfigure AP"
        );
    }


    if (configure_sta() != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to reconfigure STA"
        );
    }


    /*
     * If STA configuration exists, reconnect.
     */
    if (g_saved_config.sta_configured) {

        esp_wifi_connect();
    }
}


/* ============================================================
 * COMMAND PARSER
 * ============================================================ */

static void process_command(char *line)
{
    if (line == NULL) {
        return;
    }


    trim_line(line);


    if (line[0] == '\0') {
        return;
    }


    /*
     * --------------------------------------------------------
     * STATUS
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "STATUS") == 0) {

        print_status();
        return;
    }


    /*
     * --------------------------------------------------------
     * HELP
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "HELP") == 0 ||
        strcmp(line, "?") == 0) {

        print_help();
        return;
    }


    /*
     * --------------------------------------------------------
     * RECONNECT
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "RECONNECT") == 0) {

        router_reconnect();

        printf(
            "STA reconnect requested.\r\n"
        );

        return;
    }


    /*
     * --------------------------------------------------------
     * AP RESET
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "AP RESET") == 0) {

        if (s_config_mutex != NULL) {
            xSemaphoreTake(
                s_config_mutex,
                portMAX_DELAY
            );
        }


        esp_err_t err =
            erase_ap_configuration();


        if (s_config_mutex != NULL) {
            xSemaphoreGive(
                s_config_mutex
            );
        }


        if (err == ESP_OK) {

            restart_wifi_after_config_change();

            printf(
                "AP configuration reset.\r\n"
            );

        } else {

            printf(
                "AP reset failed: %s\r\n",
                esp_err_to_name(err)
            );
        }

        return;
    }


    /*
     * --------------------------------------------------------
     * STA RESET
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "STA RESET") == 0) {

        if (s_config_mutex != NULL) {
            xSemaphoreTake(
                s_config_mutex,
                portMAX_DELAY
            );
        }


        esp_err_t err =
            erase_sta_configuration();


        if (s_config_mutex != NULL) {
            xSemaphoreGive(
                s_config_mutex
            );
        }


        if (err == ESP_OK) {

            disable_napt();

            esp_wifi_disconnect();

            atomic_store(
                &g_router.sta_connected,
                false
            );

            atomic_store(
                &g_router.sta_rssi,
                -127
            );

            printf(
                "STA configuration reset.\r\n"
            );

        } else {

            printf(
                "STA reset failed: %s\r\n",
                esp_err_to_name(err)
            );
        }

        return;
    }


    /*
     * --------------------------------------------------------
     * COMPLETE RESET
     * --------------------------------------------------------
     */

    if (strcasecmp(line, "RESET") == 0) {

        if (s_config_mutex != NULL) {
            xSemaphoreTake(
                s_config_mutex,
                portMAX_DELAY
            );
        }


        esp_err_t err =
            erase_all_configuration();


        if (s_config_mutex != NULL) {
            xSemaphoreGive(
                s_config_mutex
            );
        }


        if (err == ESP_OK) {

            disable_napt();

            esp_wifi_disconnect();

            atomic_store(
                &g_router.sta_connected,
                false
            );

            atomic_store(
                &g_router.sta_rssi,
                -127
            );

            atomic_store(
                &g_router.sta_channel,
                0
            );

            printf(
                "Complete router configuration reset.\r\n"
            );

        } else {

            printf(
                "Reset failed: %s\r\n",
                esp_err_to_name(err)
            );
        }

        return;
    }


    /*
     * --------------------------------------------------------
     * AP CONFIGURATION
     * --------------------------------------------------------
     *
     * AP <SSID> <PASSWORD>
     */

    if (strncasecmp(line, "AP ", 3) == 0) {

        char *args = line + 3;

        char *ssid =
            strtok(args, " \t");

        char *password =
            strtok(NULL, " \t");


        if (ssid == NULL ||
            password == NULL) {

            printf(
                "Usage: AP <ssid> <password>\r\n"
            );

            return;
        }


        if (!valid_ssid(ssid)) {

            printf(
                "Invalid SSID. Length must be 1..32.\r\n"
            );

            return;
        }


        if (!valid_password(password) ||
            strlen(password) < 8U) {

            printf(
                "Invalid AP password. WPA2 password must be 8..63 characters.\r\n"
            );

            return;
        }


        if (s_config_mutex != NULL) {
            xSemaphoreTake(
                s_config_mutex,
                portMAX_DELAY
            );
        }


        memset(
            g_saved_config.ap_ssid,
            0,
            sizeof(g_saved_config.ap_ssid)
        );

        memset(
            g_saved_config.ap_password,
            0,
            sizeof(g_saved_config.ap_password)
        );


        strlcpy(
            g_saved_config.ap_ssid,
            ssid,
            sizeof(g_saved_config.ap_ssid)
        );

        strlcpy(
            g_saved_config.ap_password,
            password,
            sizeof(g_saved_config.ap_password)
        );


        g_saved_config.ap_configured = true;


        esp_err_t err =
            save_configuration();


        if (s_config_mutex != NULL) {
            xSemaphoreGive(
                s_config_mutex
            );
        }


        if (err != ESP_OK) {

            printf(
                "AP configuration save failed: %s\r\n",
                esp_err_to_name(err)
            );

            return;
        }


        restart_wifi_after_config_change();


        printf(
            "AP configuration saved.\r\n"
        );

        return;
    }


    /*
     * --------------------------------------------------------
     * STA CONFIGURATION
     * --------------------------------------------------------
     *
     * STA <SSID> <PASSWORD>
     */

    if (strncasecmp(line, "STA ", 4) == 0) {

        char *args = line + 4;

        char *ssid =
            strtok(args, " \t");

        char *password =
            strtok(NULL, " \t");


        if (ssid == NULL ||
            password == NULL) {

            printf(
                "Usage: STA <ssid> <password>\r\n"
            );

            return;
        }


        if (!valid_ssid(ssid)) {

            printf(
                "Invalid SSID. Length must be 1..32.\r\n"
            );

            return;
        }


        if (strlen(password) >
            ROUTER_PASSWORD_MAX_LEN) {

            printf(
                "Invalid password. Maximum length is 63.\r\n"
            );

            return;
        }


        if (s_config_mutex != NULL) {
            xSemaphoreTake(
                s_config_mutex,
                portMAX_DELAY
            );
        }


        memset(
            g_saved_config.sta_ssid,
            0,
            sizeof(g_saved_config.sta_ssid)
        );

        memset(
            g_saved_config.sta_password,
            0,
            sizeof(g_saved_config.sta_password)
        );


        strlcpy(
            g_saved_config.sta_ssid,
            ssid,
            sizeof(g_saved_config.sta_ssid)
        );

        strlcpy(
            g_saved_config.sta_password,
            password,
            sizeof(g_saved_config.sta_password)
        );


        g_saved_config.sta_configured = true;


        esp_err_t err =
            save_configuration();


        if (s_config_mutex != NULL) {
            xSemaphoreGive(
                s_config_mutex
            );
        }


        if (err != ESP_OK) {

            printf(
                "STA configuration save failed: %s\r\n",
                esp_err_to_name(err)
            );

            return;
        }


        restart_wifi_after_config_change();


        printf(
            "STA configuration saved.\r\n"
        );

        return;
    }


    /*
     * --------------------------------------------------------
     * UNKNOWN COMMAND
     * --------------------------------------------------------
     */

    printf(
        "Unknown command: %s\r\n",
        line
    );

    printf(
        "Type HELP for available commands.\r\n"
    );
}


/* ============================================================
 * CONSOLE TASK
 * ============================================================
 *
 * IMPORTANT:
 *
 * The old "router> router> router>" problem generally happens
 * when the prompt is printed repeatedly while processing
 * individual characters.
 *
 * This implementation is LINE BASED.
 *
 * One complete line is read.
 * One command is executed.
 * One prompt is printed.
 *
 * ============================================================ */

static void console_task(void *arg)
{
    (void)arg;

    char line[CONSOLE_BUFFER_SIZE];


    printf("\r\n");
    printf("========================================\r\n");
    printf(" ESP32 NAT ROUTER CONSOLE\r\n");
    printf(" ESP-IDF 5.1.2\r\n");
    printf("========================================\r\n");

    print_help();

    printf("router> ");
    fflush(stdout);


    while (1) {

        memset(
            line,
            0,
            sizeof(line)
        );


        /*
         * fgets() uses the ESP-IDF VFS/stdin UART.
         *
         * It waits for a complete line instead of processing
         * every character as a separate command.
         */
        if (fgets(
                line,
                sizeof(line),
                stdin
            ) != NULL) {

            trim_line(line);

            process_command(line);

            printf(
                "router> "
            );

            fflush(stdout);

        } else {

            /*
             * Prevent a broken stdin from turning into a
             * tight CPU-consuming loop.
             */
            vTaskDelay(
                pdMS_TO_TICKS(50)
            );
        }
    }
}


/* ============================================================
 * RECONNECT TASK
 * ============================================================ */

static void reconnect_task(void *arg)
{
    (void)arg;


    while (1) {

        if (!s_wifi_started ||
            !g_saved_config.sta_configured) {

            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );

            continue;
        }


        bool connected =
            atomic_load(
                &g_router.sta_connected
            );


        if (connected) {

            /*
             * Connected state.
             *
             * Do not call esp_wifi_connect().
             */
            s_reconnect_delay_ms =
                STA_RECONNECT_INITIAL_MS;

            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );

            continue;
        }


        /*
         * Wait using exponential backoff.
         */
        vTaskDelay(
            pdMS_TO_TICKS(
                s_reconnect_delay_ms
            )
        );


        if (atomic_load(
                &g_router.sta_connected)) {

            continue;
        }


        atomic_fetch_add(
            &g_router.reconnect_count,
            1
        );


        esp_err_t err =
            esp_wifi_connect();


        if (err == ESP_OK) {

            /*
             * A successful API call only means the connection
             * request was accepted.
             *
             * Actual connection is confirmed by the event.
             */
            s_reconnect_delay_ms =
                STA_RECONNECT_INITIAL_MS;

        } else {

            if (s_reconnect_delay_ms <
                STA_RECONNECT_MAX_MS) {

                s_reconnect_delay_ms *= 2U;

                if (s_reconnect_delay_ms >
                    STA_RECONNECT_MAX_MS) {

                    s_reconnect_delay_ms =
                        STA_RECONNECT_MAX_MS;
                }
            }
        }
    }
}


/* ============================================================
 * STATISTICS TASK
 * ============================================================ */

void router_stats_task(void *arg)
{
    (void)arg;

    TickType_t last_wake =
        xTaskGetTickCount();

    uint32_t last_status_log = 0;


    while (1) {

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                STA_STATS_INTERVAL_MS
            )
        );


        atomic_fetch_add(
            &g_router.uptime_sec,
            1
        );


        update_sta_statistics();


        /*
         * Keep routine logging out of the high-speed packet
         * path.
         *
         * This is intentionally very infrequent.
         */
        last_status_log++;


        if (last_status_log >=
            (STA_STATUS_LOG_INTERVAL_MS /
             STA_STATS_INTERVAL_MS)) {

            last_status_log = 0;


            /*
             * Only log a compact line.
             *
             * Full STATUS is user requested.
             */
            if (atomic_load(
                    &g_router.sta_connected)) {

                ESP_LOGD(
                    TAG,
                    "STA RSSI=%d CH=%d NAPT=%s",
                    atomic_load(
                        &g_router.sta_rssi
                    ),
                    atomic_load(
                        &g_router.sta_channel
                    ),
                    atomic_load(
                        &g_router.nat_enabled
                    ) ? "ON" : "OFF"
                );
            }
        }
    }
}


/* ============================================================
 * MANUAL RECONNECT
 * ============================================================ */

void router_reconnect(void)
{
    if (!s_wifi_started) {
        return;
    }


    if (!g_saved_config.sta_configured) {

        printf(
            "STA is not configured.\r\n"
        );

        return;
    }


    /*
     * Manual reconnect resets the backoff.
     */
    s_reconnect_delay_ms =
        STA_RECONNECT_INITIAL_MS;


    atomic_store(
        &g_router.sta_connected,
        false
    );


    disable_napt();


    esp_wifi_disconnect();


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );


    esp_err_t err =
        esp_wifi_connect();


    if (err != ESP_OK) {

        ESP_LOGW(
            TAG,
            "Manual reconnect failed: %s",
            esp_err_to_name(err)
        );
    }
}


/* ============================================================
 * ROUTER CORE INITIALIZATION
 * ============================================================ */

esp_err_t router_core_init(void)
{
    if (s_initialized) {

        return ESP_ERR_INVALID_STATE;
    }


    /*
     * --------------------------------------------------------
     * Mutex
     * --------------------------------------------------------
     */

    s_config_mutex =
        xSemaphoreCreateMutex();

    if (s_config_mutex == NULL) {

        return ESP_ERR_NO_MEM;
    }


    /*
     * --------------------------------------------------------
     * Event group
     * --------------------------------------------------------
     */

    s_router_events =
        xEventGroupCreate();

    if (s_router_events == NULL) {

        return ESP_ERR_NO_MEM;
    }


    /*
     * --------------------------------------------------------
     * Load NVS configuration
     * --------------------------------------------------------
     */

    esp_err_t err =
        load_configuration();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Configuration load failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "AP configuration: %s",
        g_saved_config.ap_configured
            ? "stored"
            : "default"
    );

    ESP_LOGI(
        TAG,
        "STA configuration: %s",
        g_saved_config.sta_configured
            ? "stored"
            : "not configured"
    );


    /*
     * --------------------------------------------------------
     * ESP-NETIF
     * --------------------------------------------------------
     */

    err =
        esp_netif_init();

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "esp_netif_init failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * --------------------------------------------------------
     * Default event loop
     * --------------------------------------------------------
     */

    err =
        esp_event_loop_create_default();

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "Event loop creation failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * --------------------------------------------------------
     * Create AP / STA netifs
     * --------------------------------------------------------
     */

    s_ap_netif =
        esp_netif_create_default_wifi_ap();

    if (s_ap_netif == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create AP netif"
        );

        return ESP_ERR_NO_MEM;
    }


    s_sta_netif =
        esp_netif_create_default_wifi_sta();

    if (s_sta_netif == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create STA netif"
        );

        return ESP_ERR_NO_MEM;
    }


    /*
     * --------------------------------------------------------
     * Configure AP static IPv4 network
     * --------------------------------------------------------
     *
     * AP:
     *
     * 192.168.4.1/24
     *
     * The default DHCP server on the AP netif will serve
     * clients from this network.
     */

    esp_netif_ip_info_t ap_ip;

    memset(
        &ap_ip,
        0,
        sizeof(ap_ip)
    );


    esp_netif_str_to_ip4(
        AP_IP,
        &ap_ip.ip
    );

    esp_netif_str_to_ip4(
        AP_GW,
        &ap_ip.gw
    );

    esp_netif_str_to_ip4(
        AP_NETMASK,
        &ap_ip.netmask
    );


    /*
     * DHCP server must be stopped before changing AP IP.
     */
    esp_netif_dhcps_stop(
        s_ap_netif
    );


    err =
        esp_netif_set_ip_info(
            s_ap_netif,
            &ap_ip
        );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "AP IP configuration failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    err =
        esp_netif_dhcps_start(
            s_ap_netif
        );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "AP DHCP start failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * --------------------------------------------------------
     * Register WiFi events
     * --------------------------------------------------------
     */

    err =
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        );

    if (err != ESP_OK) {
        return err;
    }


    err =
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        );

    if (err != ESP_OK) {
        return err;
    }


    /*
     * --------------------------------------------------------
     * Start WiFi
     * --------------------------------------------------------
     */

    err =
        start_wifi();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "WiFi start failed: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * --------------------------------------------------------
     * Console task
     * --------------------------------------------------------
     */

    BaseType_t task_ok =
        xTaskCreatePinnedToCore(
            console_task,
            "router_console",
            CONSOLE_TASK_STACK,
            NULL,
            CONSOLE_TASK_PRIORITY,
            &s_console_task,
            0
        );

    if (task_ok != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create console task"
        );

        return ESP_ERR_NO_MEM;
    }


    /*
     * --------------------------------------------------------
     * Reconnect task
     * --------------------------------------------------------
     */

    task_ok =
        xTaskCreatePinnedToCore(
            reconnect_task,
            "router_reconnect",
            RECONNECT_TASK_STACK,
            NULL,
            RECONNECT_TASK_PRIORITY,
            &s_reconnect_task,
            0
        );

    if (task_ok != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create reconnect task"
        );

        return ESP_ERR_NO_MEM;
    }


    /*
     * --------------------------------------------------------
     * Statistics task
     * --------------------------------------------------------
     *
     * Put statistics on CPU1 so it doesn't unnecessarily
     * interfere with the console/event side.
     */

    task_ok =
        xTaskCreatePinnedToCore(
            router_stats_task,
            "router_stats",
            STATS_TASK_STACK,
            NULL,
            STATS_TASK_PRIORITY,
            &s_stats_task,
            1
        );

    if (task_ok != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create statistics task"
        );

        return ESP_ERR_NO_MEM;
    }


    s_initialized = true;


    ESP_LOGI(
        TAG,
        "================================================"
    );

    ESP_LOGI(
        TAG,
        "ESP32 NAT ROUTER READY"
    );

    ESP_LOGI(
        TAG,
        "AP SSID: %s",
        g_saved_config.ap_ssid
    );

    ESP_LOGI(
        TAG,
        "AP IP: %s",
        AP_IP
    );

    ESP_LOGI(
        TAG,
        "STA configured: %s",
        g_saved_config.sta_configured
            ? "YES"
            : "NO"
    );

    ESP_LOGI(
        TAG,
        "================================================"
    );


    return ESP_OK;
}
