/**
 * Library for cooperative algorithms
 * 
 * This (coop.h) and state_machine.h, are libraries where 
 * all written drivers are used to do something.
 * 
 * This library was meant for cooperative algorithms, but
 * some cooperative algorithms are implemented in 
 * state_machine.h as states.
 * 
 */

#ifndef COOP_H
#define COOP_H

// ESP-IDF libraries
#include "freertos/FreeRTOS.h"
#include "esp_random.h"

// Personal libraries
#include "io_define.h"
#include "hwtimer.h"
#include "adc_lib.h"
#include "led_driver.h"
#include "servo_driver.h"
#include "dm_comm.h"

#define TIMER_DIS 2         // Timer for periodically detecting obstacle
#define TIMER_DIS_RESOLUTION    10000  // Timer resolution in Hz
#define DIS_PERIOD 300         // With resolution 10000 Hz, 300 corresponds to 30ms
#define DIS_THRESHOLD 4000     // Threshold for deciding presence of obstacle 

#define SERVO_MOVE_SPEED 300


//----------    RANDOM WALK     ----------

#define MIN_MOVE_TIME_US    500000/BIT_DURATION_US   // 0.5 seconds
#define MAX_MOVE_TIME_US    3 * 1000000/BIT_DURATION_US  // 3 seconds
#define MAX_ROTATE_TIME_US  1 * 1000000/BIT_DURATION_US

// Duration of moving forward for leader
#define LEADER_MIN_MOVE_TIME_US 10 * 1000000/BIT_DURATION_US    
#define LEADER_MAX_MOVE_TIME_US 15 * 1000000/BIT_DURATION_US


typedef enum {
    MOVE_FORWARD,
    TURN_LEFT,
    TURN_RIGHT
} move_type_t;


// ----------   CMD2 - SPREAD OUT   ------------

// Define durations in microseconds
#define TURN_AWAY_TIME_US 1.5 * 1000000/BIT_DURATION_US
#define FORWARD_TIME_US   7 * 1000000/BIT_DURATION_US  // this going to run 2x
#define REVERSE_TIME_US   2*SERVO_ROTATE_RIGHT  // in ms

// States
typedef enum {
    STATE_TURN_AWAY,
    STATE_FORWARD_1,
    STATE_REVERSE,
    STATE_FORWARD_2,
    STATE_DONE
} spread_out_state_t;


// ----------   CHAIN FORMATION   ------------

#define SIGNAL_SAMPLE_COUNT         50      // Amount of samples read before determining position

#define COOLDOWN_AFTER_MOVE         (3*CHAIN_FORWARD_ROTATE_MS + CHAIN_FORWARD_TIME_MS)  // Wait time after back robot moves
#define CHAIN_FORWARD_ROTATE_MS     2000
#define CHAIN_FORWARD_TIME_MS       5000

#define MSG_PRESENCE_BEACON 0b1100

#define ID_UNKNOWN  0
#define ID_FRONT    1
#define ID_MIDDLE   2
#define ID_BACK     3


/**
 * @brief Initialize obstacle detection (ADC and LEDs)
 * 
 * @param adc1_ch   Channels of ADC1
 * @param a1_size   Number of ADC1 channels
 * @param leds      LED pins
 * @param l_size    Number of LEDs
 * 
 */
void coop_dis_init(adc1_channel_t *adc1_ch, int a1_size, gpio_num_t *leds, int l_size);

/**
 * @brief Checks if obstacle is detected 
 * 
 * @return If obstacle detected return "1" (HIGH)
 */
bool coop_obstacle_detection(void);

/**
 * @brief Get direction of strongest signal
 * 
 * @param adc_values Read values 
 * 
 * @return Returns direction of strongest signal
 */
int coop_signal_direction(int adc_values[CHANNEL_NUM]);

/**
 * @brief Turn to chosen direction (usually strongest)
 * 
 * @param direction     Chosen direction
 * 
 */
void coop_turn_to_signal(int direction);

/**
 * @brief Turn away from chosen direction (usually strongest)
 * 
 * @param direction     Chosen direction
 * 
 */
void coop_turn_away(int direction);


/**
 * @brief Start random walk 
 * 
 * Resets variables.
 */
void random_walk_start();

/**
 * @brief Apply chosen movement
 * 
 * @param move  Chosen movement
 * 
 */
void apply_move(move_type_t move);

/**
 * @brief Randomly choose next movement
 * 
 * @param time_now Current time
 * @param state    State of robot (state machine)
 * 
 */
void random_walk_choose(uint32_t time_now, int state);

/**
 * @brief Loop for random walk
 * 
 * Checks time and based on it switches to a different movement.
 * 
 * @param time_now Current time
 * @param state    State of robot (state machine)
 * 
 */
void random_walk_loop(uint32_t time_now, int state);

/**
 * @brief Find signal to decode 
 * 
 * The robot stops when signal is found, but because of inertia might not stop properly.
 */
void signal_correction();


/**
 * @brief Start algorithm "Spread out" - COMMAND2
 * 
 * The robots turn away from leader (beacon) and go further away.
 * After some time robots turn around and go back.
 */
void coop_start_spread();

/**
 * @brief Loop do determine current movement
 * 
 * @param time_now      Current time
 * @param flag_close    If "1", then it's too close -> back off
 * 
 */
void coop_spread_out_loop(uint32_t time_now, bool flag_close);

/**
 * @brief Get current movement
 * 
 * @return Returns current movement
 */
uint8_t coop_get_state();


/**
 * @brief Algorithm "CHAIN" 
 * 
 * The robots have to be aligned behind each other and turned in one
 * direction to front (the robots looks at the back of robots in front)
 * 
 * The robots sends messages and based on the direction of messages 
 * determines position (back, middle, front). If the robot is in the 
 * back, it moves to the front of line. Otherwise stays in place.
 * 
 * DOWNSIDE - Dependent on actual movement of robot (the robot have to really go straight in line and turn 90°)
 * 
 */
void state_chain();

#endif