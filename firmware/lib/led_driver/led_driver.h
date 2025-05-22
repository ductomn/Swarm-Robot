/**
 * Library for LEDs
 */


#ifndef LED_DRIVER_H
#define LED_DRIVER_H

// ESP-IDF libraries
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Personal libraries
#include "io_define.h"

/**
 * @brief Initialize pin for LED
 * 
 * @param pin    Chosen GPIO pin
 * 
 */
void led_init(gpio_num_t pin);

/**
 * @brief Initialize multiple pins for LEDs
 * 
 * @param pins      Array of GPIO pins
 * @param num_pins  Number of pins
 * 
 */
void multiple_led_init(const gpio_num_t *pins, int num_pins);

/**
 * @brief Drive LED to chosen state
 * 
 * @param pin    Chosen GPIO pin
 * @param state  "1" (HIGH) or "0" (LOW)
 * 
 */
void led_drive(gpio_num_t pin, int state);

/**
 * @brief Toggle LED
 * 
 * Changes state to opposite of current
 * 
 * @param pin    Chosen GPIO pin
 * 
 */
void led_toggle(gpio_num_t pin);

/**
 * @brief Drive multiple LEDs to chosen state
 * 
 * @param pins      Array of GPIO pins
 * @param num_pins  Number of pins
 * @param state     "1" (HIGH) or "0" (LOW)
 * 
 */
void multiple_led_drive(const gpio_num_t *pins, int num_pins, int state);

#endif // LED_DRIVER_H
