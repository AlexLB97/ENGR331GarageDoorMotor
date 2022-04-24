/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for managing the various events that can occur in the system.
 */

#ifndef SYSTEM_EVENTS_H
#define SYSTEM_EVENTS_H

typedef enum system_event_t {
    EVENT_NONE = 0,
    EVENT_NEXT_DOOR_STATE,
    EVENT_STOP,
    EVENT_MAX_EVENT_NUM
} system_event_t;

#endif
