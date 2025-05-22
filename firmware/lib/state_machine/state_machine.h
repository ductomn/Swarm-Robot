/**
 * Library for state machine
 * 
 * This (state_machine.h) and coop.h, are libraries where 
 * all written drivers are used to do something.
 * 
 * Each function represents a state. Obstacle avoidance is run
 * all the time, except for some states. 
 * 
 * Some algorithms simulates calling helpers. The robots go around
 * randomly and someone starts sending messages (COMMANDx_SIG).
 * The robots do tasks based on received messages and after a while
 * goes back to random walk. The leader can be determined beforehand
 * or randomly. For determining leader beforehand set macro 
 * WITH_LEADER to "1" in io_define.h.
 * 
 * Other algorithms runs right after turning on (CHAIN, COMMAND3/FOLLOW_CHAIN). 
 * These algorithm needs to have some conditions fulfilled to work,
 * and these conditions can't be fulfilled while in random walk.
 * To turn on these algorithms set corresponding macros to "1". 
 * 
 * COMMAND3 can be run after random walk, but has a high chance 
 * for failure.
 * 
 * state_chain is implemented in coop.h, coop.c.
 * 
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// C/C++ libraries
#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"

// ESP-IDF libraries
#include "freertos/FreeRTOS.h"
#include "esp_random.h"

// Personal libraries
#include "dm_comm.h"
#include "io_define.h"
#include "servo_driver.h"
#include "coop.h"


#define MIN_IDLE_TIME   3000    // Minimal IDLE time
#define RAND_IDLE_TIME  3000    // Additional IDLE time chosen randomly

#define MIN_WALK_TIME   4000    // Minimal WALK time
#define RAND_WALK_TIME  5000    // Additional WALK time chosen randomly

#define LISTEN_TIME     2000    // LISTEN time before going back to random walk

typedef enum {
    IDLE,
    RANDOM_WALK,
    LISTEN,
    TRANSMITTING,
    COMMAND_RECEIVED,
    COMMAND1,
    COMMAND2,
    COMMAND3,
    CHAIN_FORMATION,
} CommState;

/**
 * @brief Infinite loop 
 * 
 * while(1) loop where all states are run.
 * 
 */
void state_machine_loop();

/**
 * @brief Initialize state machine
 */
void state_machine_init();


/**
 * @brief State IDLE
 * 
 * This state is run after turning robot on. 
 * Robots can calibrate during this state (not implemented).
 * 
 * Transitions to RANDOM_WALK, CHAIN or COMMAND3 (FOLLOW_CHAIN).
 *
 * 
 */
void state_idle();

/**
 * @brief State RANDOM_WALK
 * 
 * The robot goes around randomly and looks for signals.
 * If randomly generated time runs out, the robot starts
 * sending signals. 
 * 
 * Transitions to LISTEN or TRANSMITTING.
 * 
 */
void state_random_walk();

/**
 * @brief State LISTEN
 * 
 * Robot stays still and listens and decodes received messages.
 * 
 * Transitions to COMMAND_RECEIVED.
 * 
 */
void state_listen();

/**
 * @brief State TRANSMITTING
 * 
 * Robot stays still, chooses task (COMMANDx) and sends messages.
 * 
 * Transitions to chosen COMMANDx.
 * 
 */
void state_transmitting();

/**
 * @brief State COMMAND_RECEIVED
 * 
 * Based on received message transitions to COMMANDx.
 * This state can be removed.
 * 
 * Transitions to COMMANDx.
 * 
 */
void state_command_received();

/**
 * @brief State COMMAND1 - follow
 * 
 * In this state algorithm "Follow" is implemented.
 * If the robot is a leader, the robot emits IR light and goes randomly.
 * If robot is a follower, it will follow the leader.
 * 
 * DOWNSIDE - Followers don't care about suroundings
 * 
 */
void state_command1();

/**
 * @brief State COMMAND2 - spread out 
 * 
 * In this state algorithm "Spread out" is implemented.
 * If the robot is a leader, the robot emits IR light and stays still.
 * If robot is a follower, it will go away from leader and later come back, simulates searching mission.
 * 
 * DOWNSIDE - Dependent on actual movement of robot (the robot have to really go straight in line and turn 180°)
 * 
 */
void state_command2();


/**
 * @brief State COMMAND3 - follow chain
 * 
 * In this state algorithm "Follow chain" is implemented.
 * The robots follow each other in a chain, based on assigned ID.
 * IDs can be assigned beforehand or dynamically. Right now it's assigned beforehand.a64l
 * Dynamic ID assigned might not work properly, not tested properly.
 * 
 * 
 * DOWNSIDE - All robots with smaller ID have to be present. 
 * 
 */
void state_command3_chain();
 

/**
 * @brief Clear COMMANDx
 * 
 * Clear COMMANDx and transition back to random walk.
 * 
 */
void state_command_clear();

/**
 * @brief Avoid obstacle
 * 
 * Avoid obstacle, but can do other stuff depending on current state (COMMANDx).
 * 
 */
void obstacle_avoidance();

#endif