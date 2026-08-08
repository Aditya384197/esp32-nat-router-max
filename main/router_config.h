#ifndef ROUTER_CONFIG_H
#define ROUTER_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_wifi.h"

/*
 * ============================================================
 * ESP32 NAT ROUTER
 * Persistent configuration + runtime configuration
 * ESP-IDF v5.1.2 / ESP32 Classic
 * ============================================================
 */

#define ROUTER_NVS_NAMESPACE        "router"

/* ============================================================
 * NVS KEYS
 * ============================================================ */

#define NVS_KEY_AP_SSID             "ap_ssid"
#define NVS_KEY_AP_PASS             "ap_pass"
#define NVS_KEY_STA_SSID            "sta_ssid"
#define NVS_KEY_STA_PASS            "sta_pass"

/* ============================================================
 * STRING LIMITS
 * ============================================================ */

#define ROUTER_SSID_MAX_LEN         32U
#define ROUTER_PASSWORD_MAX_LEN     63U

/* ============================================================
 * DEFAULT AP
 * ============================================================ */

#define AP_DEFAULT_SSID             "ESP32_Router"
#define AP_DEFAULT_PASSWORD         ""

#define AP_IP                       "192.168.4.1"
#define AP_GW                       "192.168.4.1"
#define AP_NETMASK                  "255.255.255.0"

#define AP_MAX_CONN                 5U
#define AP_INITIAL_CHANNEL          1U

/* ============================================================
 * PERFORMANCE
 * ============================================================ */

#define ROUTER_CPU_FREQ_MHZ         240U

#define ROUTER_WIFI_PS              WIFI_PS_NONE

/*
 * esp_wifi_set_max_tx_power()
 *
 * Unit = 0.25 dBm.
 * 80 = 20 dBm.
 */
#define ROUTER_MAX_TX_POWER         80

/* ============================================================
 * STA RECONNECT
 * ============================================================ */

#define STA_RECONNECT_INITIAL_MS    500U
#define STA_RECONNECT_MAX_MS        16000U

#define STA_CONNECT_WAIT_MS         15000U

/* ============================================================
 * STATISTICS
 * ============================================================ */

#define STA_STATS_INTERVAL_MS       1000U
#define STA_STATUS_LOG_INTERVAL_MS  5000U

/* ============================================================
 * SERIAL CONSOLE
 * ============================================================ */

#define ROUTER_CONSOLE_STACK_SIZE   4096U
#define ROUTER_CONSOLE_PRIORITY     5U

#define ROUTER_CONSOLE_LINE_MAX     128U

#define ROUTER_CONSOLE_PROMPT       "router> "

/*
 * UART input is normally provided through stdin
 * by the ESP-IDF console/monitor environment.
 *
 * We deliberately do NOT print the prompt from the
 * low-level read loop repeatedly when stdin has no data.
 */
#define ROUTER_CONSOLE_POLL_MS      50U

/* ============================================================
 * RUNTIME STATE
 * ============================================================ */

typedef struct
{
    bool sta_connected;

    int sta_rssi;

    int sta_channel;

    int ap_clients;

    bool nat_enabled;

    uint32_t uptime_sec;

    uint32_t reconnect_count;

    uint32_t napt_failures;

} router_config_t;


/*
 * Defined exactly once in router_core.c.
 */
extern router_config_t g_router;

/* ============================================================
 * SAVED CONFIGURATION
 * ============================================================ */

typedef struct
{
    char ap_ssid[ROUTER_SSID_MAX_LEN + 1U];

    char ap_password[ROUTER_PASSWORD_MAX_LEN + 1U];

    char sta_ssid[ROUTER_SSID_MAX_LEN + 1U];

    char sta_password[ROUTER_PASSWORD_MAX_LEN + 1U];

    bool ap_configured;

    bool sta_configured;

} router_saved_config_t;


/*
 * Defined exactly once in router_core.c.
 */
extern router_saved_config_t g_saved_config;

#endif /* ROUTER_CONFIG_H */
