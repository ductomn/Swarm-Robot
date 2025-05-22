/**
 * Library for useful defines
 * 
 * Includes flags for different kinds of modes (WITH_LEADER, CHAIN)
 */


#ifndef IO_DEFINE_H
#define IO_DEFINE_H

#include "driver/gpio.h"
#include "driver/adc.h"

#define ROBOT_ID 4

#define WITH_LEADER 1

#define FOLLOW_CHAIN 0 // if set to 1, CHAIN has to be set to 0

#define CHAIN 0

// Number of ADC channels
#define CHANNEL_NUM 6       // Communication
#define DIS_CHANNEL_NUM 2   // Distance measuring / obstacle detection

#define GET_SIZE(x) sizeof(x) / sizeof(x[0])

// "Macros" for each pin
typedef enum
{
    IO_TEST_LED = GPIO_NUM_2,   // this is NOT on the robot
    
    IO_IR_SIG = GPIO_NUM_14,
    IO_IR_DIS = GPIO_NUM_27,

    IO_DIS_LEFT = ADC1_CHANNEL_5,
    IO_DIS_RIGHT = ADC1_CHANNEL_6,
    IO_SIG_FRONT = ADC1_CHANNEL_4,
    IO_SIG_FRONT_LEFT = ADC2_CHANNEL_9,
    IO_SIG_FRONT_RIGHT = ADC1_CHANNEL_7,
    IO_SIG_BACK = ADC2_CHANNEL_0,
    IO_SIG_BACK_LEFT = ADC2_CHANNEL_4,
    IO_SIG_BACK_RIGHT = ADC2_CHANNEL_8,

    IO_MOTOR_RIGHT = GPIO_NUM_17,
    IO_MOTOR_LEFT = GPIO_NUM_18,
    IO_UART_RX = GPIO_NUM_3,
    IO_UART_TX = GPIO_NUM_1,
    IO_SPI_SCL = GPIO_NUM_22,
    IO_SPI_SDA = GPIO_NUM_21,

} io_e;



#if WITH_LEADER
    #if (ROBOT_ID == 1)
        #define LEADER 1
    #else
        #define LEADER 0
    #endif
#endif

// Each robot drives servomotors differently and the wheels might slip, 
// these are modifiers for speed 300.
// Note that these might not be correct, based on the surface
#if (ROBOT_ID == 1)
    #define SERVO_FORWARD_LEFT_MOD    -50
    #define SERVO_BACKWARDS_RIGHT_MOD -120
    #define SERVO_ROTATE_RIGHT      855
    #define SERVO_ROTATE_LEFT       900
#elif (ROBOT_ID == 2)
    #define SERVO_FORWARD_LEFT_MOD    -80
    #define SERVO_BACKWARDS_RIGHT_MOD +60
    #define SERVO_ROTATE_RIGHT      740
    #define SERVO_ROTATE_LEFT       700
#elif (ROBOT_ID == 3)
    #define SERVO_FORWARD_LEFT_MOD    0
    #define SERVO_BACKWARDS_RIGHT_MOD -60
    #define SERVO_ROTATE_RIGHT      910
    #define SERVO_ROTATE_LEFT       790
#elif (ROBOT_ID == 4)
    #define SERVO_FORWARD_LEFT_MOD    -70
    #define SERVO_BACKWARDS_RIGHT_MOD +30
    #define SERVO_ROTATE_RIGHT      810
    #define SERVO_ROTATE_LEFT       850
#else
    #define SERVO_FORWARD_LEFT_MOD    0
    #define SERVO_BACKWARDS_RIGHT_MOD 0
    #define SERVO_ROTATE_RIGHT      855
    #define SERVO_ROTATE_LEFT       900
#endif

#endif