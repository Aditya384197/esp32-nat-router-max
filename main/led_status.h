#ifndef LED_STATUS_H
#define LED_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize router status LED.
 */
void led_init(void);

/**
 * LED OFF.
 */
void led_off(void);

/**
 * LED ON.
 */
void led_on(void);

/**
 * Toggle LED state.
 */
void led_toggle(void);

/**
 * Set LED according to router state.
 *
 * connected = true  -> connected/ready indication
 * connected = false -> disconnected indication
 */
void led_set_connected(int connected);

#ifdef __cplusplus
}
#endif

#endif /* LED_STATUS_H */
