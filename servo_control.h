/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for controlling the servo motor
 */


#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

/* Type Definitions */
typedef enum door_state_t {
    DOOR_STATE_OPENING,
    DOOR_STATE_CLOSING,
    DOOR_STATE_STOPPED,
    DOOR_STATE_CLOSED,
    DOOR_STATE_OPEN
} door_state_t;


/**
 * @brief Function that manages the sequence of state for the door and returns the next state based
 * on current and previous state.
 */
door_state_t servo_control_get_next_state(void);


/**
 * @brief Function that handles the door's transition to the next state based on
 * the current and previous state.
 */
void servo_control_handle_state_transition(door_state_t next_state);

/**
 * @brief Getter to allow other modules access to motor state
 * 
 * @param [out] current_state Returns the current door state
 */
door_state_t servo_control_get_current_state(void);


void servo_init(void);


#endif
