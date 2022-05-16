/**
 * Authors: Alex Bourdage, Sophie Woessner
 * 
 * This module is responsible for keeping track of the interior lighting conditions and toggling
 * the interior light on and off accordingly.
 */

#ifndef INTERIOR_LIGHTING_H
#define INTERIOR_LIGHTING_H

#include "motion_detector.h"


typedef enum {
    LIGHT_ON = 0,
    LIGHT_OFF
} interior_lighting_state_t;

void interior_lighting_init(void);

#endif

