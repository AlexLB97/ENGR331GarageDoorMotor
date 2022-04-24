/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module to control the opening and closing operations of the DC motor that simulates the 
 * garage door opener
 */
 
 #ifndef MOTOR_CONTROL_H
 #define MOTOR_CONTROL_H

/* Includes */
#include "system_events.h"

/* Type Definitions */
typedef enum door_state_t {
    DOOR_STATE_OPENING,
    DOOR_STATE_CLOSING,
    DOOR_STATE_STOPPED,
    DOOR_STATE_UNKNOWN
} door_state_t;

/* Function declarations */

/**
 * @brief Function to initialize the DC motor and the state of the motor control module.
 */
void motor_control_init(void);

/**
 * @brief Function that manages the sequence of state for the door and returns the next state based
 * on current and previous state.
 */
door_state_t motor_control_get_next_state(void);


/**
 * @brief Function that handles the door's transition to the next state based on
 * the current and previous state.
 */
void motor_control_handle_state_transition(door_state_t next_state);

/**
 * @brief Getter to allow other modules access to motor state
 * 
 * @param [out] current_state Returns the current door state
 */
door_state_t motor_control_get_current_state(void);

#endif
