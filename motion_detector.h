/**
 * Alex Bourdage
 * Sophie Woessner
 * Goal: Module for initializing and handling input from the motion sensor.
 */

#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

/* Type Definitions */

typedef enum occupancy_state_t {
    GARAGE_OCCUPIED,
    GARAGE_UNOCCUPIED
} occupancy_state_t;


occupancy_state_t motion_detector_get_occupancy_state(void);


/**
 * @brief Function for initializing the motion detector as an input with an interrupt
 */
void motion_detector_init(void);



#endif
