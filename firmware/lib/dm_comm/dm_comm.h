/**
 * Library for communication using Differential Manchester encoding
 * 
 * Includes sending and receiving messages. 
 * Messages are received continuously with chosen interval.
 * 
 */


#ifndef DM_COMM_H
#define DM_COMM_H

// ESP-IDF libraries
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

// Personal libraries
#include "io_define.h"
#include "adc_lib.h"
#include "hwtimer.h"
#include "led_driver.h"


#define MSG_LENGTH 4            // Number of bits per message
#define BIT_DURATION_US 1000    // Duration of half clock cycle (for Differential Manchester encoding)
#define SIG_THRESHOLD 500       // If higher read as "1" (HIGH)
#define START_SIG 0b1110        // Sent before every msg (for synchronisation)
#define START_SIG_LEN 4         // Number of bits for START_SIG

#define CYCLE_BIT_COUNT     (2*MSG_LENGTH + START_SIG_LEN)      // Total number of bits per sent/received message
#define MSG_INTERVAL        (CYCLE_BIT_COUNT* 2)                // Wait time before sending next message

#define MSG_TIME_TAKEN      (MSG_INTERVAL * BIT_DURATION_US)    
#define MAX_SEND_COUNT      5 * 1000000/MSG_TIME_TAKEN       // sending for 5s
#define COMMAND_COUNT       3               // Least ammount of received COMMAND_SIG to commence

#define COMMAND1_SIG        0b0001      // Message for commencing COMMAND1
#define COMMAND2_SIG        0b0010      // Message for commencing COMMAND2
#define COMMAND3_SIG        0b0100      // Message for commencing COMMAND3

#define CMD_START_SIG       0b1011    // signal to commence commanded action
#define COMMAND_PERIOD      50 * 1000000 / BIT_DURATION_US     // Duration of "work" time
#define LEADER_BACKOFF      5 * 1000000 /  BIT_DURATION_US     // additional wait time for current leader, to let others lead


#define MIN_BACKOFF_CYCLE   (COMMAND_PERIOD + MAX_SEND_COUNT/2)
#define MAX_BACKOFF_CYCLE   2 * 1000000 / (CYCLE_BIT_COUNT * BIT_DURATION_US)     // duration of backoff (MAX_BACKOFF_CYCLE * (2*MSG_LENGTH + START_SIG_LEN))


/**
 * @brief Initialize communication (ADC and LEDs)
 * 
 * @param adc1_ch   Channels of ADC1
 * @param a1_size   Number of ADC1 channels
 * @param adc2_ch   Channels of ADC2
 * @param a2_size   Number of ADC2 channels
 * @param leds      LED pins
 * @param l_size    Number of LEDs
 * 
 */
void dm_comm_init(adc1_channel_t *adc1_ch, int a1_size, adc2_channel_t *adc2_ch, int a2_size, gpio_num_t *leds, int l_size);

/**
 * @brief Start timer interrupt for communication
 */
void dm_comm_start(void);

/**
 * @brief Stop timer interrupt for communication
 */
void dm_comm_stop(void);

/**
 * @brief Send message
 * 
 * @param message   Message to be sent
 * 
 */
void dm_comm_send(int message);

/**
 * @brief Stop reading 
 * 
 * There is a flag inside timer callback.
 */
void dm_comm_reading_stop(void);

/**
 * @brief Start reading 
 * 
 * There is a flag inside timer callback.
 */
void dm_comm_reading_start(void);

/**
 * @brief Decode message from chosen channel
 * 
 * @param i    Chosen channel
 * 
 */
void decode_channel(int i);

/**
 * @brief Checks if START_SIG is detected 
 * 
 * @return If START_SIG detected return "1" (HIGH)
 */
bool dm_comm_detect_start_sig();

/**
 * @brief Message is processed/decoded
 * 
 * Checks and tries to decode messages from all channels.
 * 
 * @return If message decoded succesfully return "1" (HIGH)
 */
bool dm_comm_process(void);

/**
 * @brief Checks if signal is detected 
 * 
 * Simply checks presence of signal.
 * 
 * @return If signal higher than threshold return "1" (HIGH)
 */
bool dm_comm_detect_signals(void);

/**
 * @brief Get signals from all channels
 * 
 * @param adc_results   Array for read signals
 * 
 */
void dm_comm_get_signals(int adc_results[CHANNEL_NUM]);

/**
 * @brief Get messages from all channels
 * 
 * @param rx_msg    Array for decoded messages
 * 
 */
void dm_comm_get_messages(int rx_msg[CHANNEL_NUM]);

/**
 * @brief Get strength of message
 * 
 * This function was not tested properly.
 * It's supposed to to return strength 
 * of signals from decoded messages.
 * 
 * @param adc_results   Array for read signals
 * 
 */
void dm_comm_get_msg_strength(int adc_results[CHANNEL_NUM]);

/**
 * @brief Checks if robot is waiting for backoff time 
 * 
 * It's for CSMA/CA. The robot can't send messages if 
 * backoff is active.
 * 
 * @return Returns state of backoff flag
 */
bool dm_comm_backoff(void);

/**
 * @brief Sets backoff time
 * 
 * @param backoff   Backoff time
 * 
 */
void dm_comm_set_backoff(int backoff);

#endif