
/**
 * Library for multiple hardware general purpose timers
 * 
 * I think ESP32 has just 4 timers, so MAX_TIMERS 
 * in hwtimer.c is set to 4. Adjust if needed/can.
 * 
 */



#ifndef HWTIMER_H
#define HWTIMER_H

// C/C++ libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ESP-IDF libraries
#include "esp_err.h"
#include "hal/timer_types.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"


#define MAX_TIMERS 4  // Adjust based on needs
#define CLOCK_TIMER 0


typedef void (*timer_callback_t)(void);  // Function pointer type for user callback


/**
 * @brief Initialize the hardware timer that periodically reloads (repeats).
 *
 * This function initializes the hardware timer that periodically reloads (repeats).
 * 
 * @param timer_id  Chosen timer
 * @param resolution    Counter resolution (working frequency) in Hz,
                        hence, the step size of each count tick equals to (1 / resolution_hz) seconds
 * @param timer_count   Alarm target count value (for resolution=1000000 timer_interval is period in microseconds)
 * @param callback  A callback function that is called each time timer_count is reached.
 *                  Pass NULL if you don't need callbacks.
 *
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t hwtimer_init(int timer_id, uint32_t resolution, uint64_t timer_count, timer_callback_t callback);


/**
 * @brief Start hardware timer
 * 
 * @param timer_id  Chosen timer
 */
void hwtimer_start(int timer_id);

/**
 * @brief Stop hardware timer
 * 
 * @param timer_id  Chosen timer
 */
void hwtimer_stop(int timer_id);

/**
 * @brief Deinitialize hardware timer
 * 
 * @param timer_id  Chosen timer
 */
void hwtimer_deinit(int timer_id);


/**
 * @brief Initialize the hardware timer that runs just once (oneshot) --> NOT TESTED PROPERLY
 * 
 * @param timer_id  Chosen timer
 * @param resolution    Counter resolution (working frequency) in Hz,
                        hence, the step size of each count tick equals to (1 / resolution_hz) seconds
 * @param timer_count   Alarm target count value (for resolution=1000000 timer_interval is period in microseconds)
 * @param callback  A callback function that is called each time timer_count is reached.
 *                  Pass NULL if you don't need callbacks.
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t hwtimer_once_init(int timer_id, uint32_t resolution, uint64_t timer_count, timer_callback_t callback);

/**
 * @brief Start oneshot hardware timer
 * 
 * @param timer_id  Chosen timer
 */
void hwtimer_once_start(int timer_id);

/**
 * @brief Initialize the hardware timer that acts as a clock
 * 
 * Contains 2 iteration (time) tracking variables, can be accessed and reseted independently
 * 
 * @param resolution    Counter resolution (working frequency) in Hz,
                        hence, the step size of each count tick equals to (1 / resolution_hz) seconds
 * @param timer_count   Alarm target count value (for resolution=1000000 timer_interval is period in microseconds)
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t hwtimer_clock_init(uint32_t resolution, uint64_t timer_count);


/**
 * @brief Get time from 1. clock
 * Clock used to track time of each state
 * 
 * @return Returns number of completed iteration of timer
 */
uint32_t hwtimer_get_time();

/**
 * @brief Resets 1. clock (iteration/time tracking variable)
 * 
 */
void hwtimer_reset_clock();

/**
 * @brief Get time from 2. clock 
 * Clock used for switching between states in random walk
 * 
 * @return Returns number of completed iteration of timer
 */
uint32_t hwtimer_cmd_get_time();

/**
 * @brief Resets 2. clock (iteration/time tracking variable)
 * 
 */
void hwtimer_cmd_reset_clock();

#endif // HWTIMER_H