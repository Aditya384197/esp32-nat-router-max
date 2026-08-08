#ifndef ROUTER_CORE_H
#define ROUTER_CORE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * ============================================================
 * ROUTER CORE PUBLIC API
 * ============================================================
 */

/**
 * @brief Initialize the complete NAT router.
 *
 * Initializes:
 *
 *   - ESP-NETIF
 *   - default event loop
 *   - AP interface
 *   - STA interface
 *   - Wi-Fi driver
 *   - DHCP server
 *   - STA configuration
 *   - NAPT
 *   - reconnect handling
 *   - statistics
 *   - serial console
 *
 * Configuration is loaded from NVS.
 */
esp_err_t router_core_init(void);


/**
 * @brief Request a manual STA reconnect.
 *
 * Safe to call even when STA is not configured.
 */
void router_reconnect(void);


/**
 * @brief Router statistics task.
 *
 * Public for compatibility with the existing architecture.
 */
void router_stats_task(void *arg);


/**
 * @brief Router console task.
 *
 * Handles commands such as:
 *
 *   AP
 *   STA
 *   AP RESET
 *   STA RESET
 *   RESET
 *   STATUS
 *
 * This function is normally started internally by
 * router_core_init().
 */
void router_console_task(void *arg);


/**
 * @brief Print current router status.
 */
void router_print_status(void);


/**
 * @brief Load router configuration from NVS.
 */
esp_err_t router_config_load(void);


/**
 * @brief Save router configuration to NVS.
 */
esp_err_t router_config_save(void);


/**
 * @brief Erase AP configuration.
 */
esp_err_t router_config_reset_ap(void);


/**
 * @brief Erase STA configuration.
 */
esp_err_t router_config_reset_sta(void);


/**
 * @brief Erase complete router configuration.
 */
esp_err_t router_config_reset_all(void);

#ifdef __cplusplus
}
#endif

#endif /* ROUTER_CORE_H */
