/**
 * Library for Servomotors
 * 
 * Set functions for chosen movements with chosen speed
 * Robots don't have encoders and the wheels might slip,
 * which actually do, so the robot won't go in straight
 * line if both wheels are set to same speed.
 * 
 * The speed is calibrated for each robot, if used on 
 * different surface than it was calibrated for, might 
 * not work.
 * 
 */


#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

// ESP-IDF libraries
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Personal libraries
#include "io_define.h"
#include "hwtimer.h"
#include "adc_lib.h"


#define SERVO_LEFT_GPIO  IO_MOTOR_LEFT  // Left servo pin
#define SERVO_RIGHT_GPIO IO_MOTOR_RIGHT  // Right servo pin

#define SERVO_LEFT_CHANNEL LEDC_CHANNEL_0
#define SERVO_RIGHT_CHANNEL LEDC_CHANNEL_1

// Macros for configuration
#define SERVO_FREQ       50  // 50Hz PWM frequency (20ms period)
#define SERVO_TIMER      LEDC_TIMER_0
#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_RESOLUTION LEDC_TIMER_13_BIT

// Pulse width limits for the servo (values in microseconds)
#define SERVO_MIN_US     1000  // Full reverse
#define SERVO_NEUTRAL_US 1500  // Stop
#define SERVO_MAX_US     2000  // Full forward

// Convert microseconds to LEDC duty cycle
#define SERVO_DUTY(us)   ((us) * (1 << SERVO_RESOLUTION) / 20000) 

#define GET_SIZE(x) sizeof(x) / sizeof(x[0])


#define SERVO_ROTATE_LEFT_SPEED   190
#define SERVO_ROTATE_RIGHT_SPEED  160

/**
 * @brief Initialize servomotor
 * 
 * @param channel    PWM channel
 * @param gpio       GPIO pin of servomotor
 */
void servo_init(ledc_channel_t channel, int gpio);

/**
 * @brief Set speed (duty cycle) of servomotor
 * 
 * @param channel    PWM channel
 * @param speed      Ranges from -1000 (backwards) to 1000 (forward), 0 (stop)                  
 */
void servo_set_speed(ledc_channel_t channel, int speed);

/**
 * @brief Move forward with chosen speed
 * 
 * Calibrated for each robot
 *
 * @param speed      Ranges from 0 (stop) to 1000 (full speed)                  
 */
void servo_move_forward(int speed);

/**
 * @brief Move backwards with chosen speed
 * 
 * Calibrated for each robot
 *
 * @param speed      Ranges from 0 (stop) to 1000 (full speed)                 
 */
void servo_move_backwards(int speed);

/**
 * @brief Rotate right with chosen speed
 *
 * @param speed      Ranges from 0 (stop) to 1000 (full speed)                 
 */
void servo_rotate_right(int speed);

/**
 * @brief Rotate left with chosen speed
 *
 * @param speed      Ranges from 0 (stop) to 1000 (full speed)                 
 */
void servo_rotate_left(int speed);

/**
 * @brief Rotate right by about 90°, mby a lil more            
 */
void servo_rotate_right_91();   // Rotate right for a little more than 90°

/**
 * @brief Rotate left by about 90°, mby a lil more            
 */
void servo_rotate_left_91();    // Rotate left for a little more than 90°

/**
 * @brief Stop          
 */
void servo_stop();

#endif