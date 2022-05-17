/**
 * Authors: Alex Bourdage and Sophie Woessner
 * 
 * Goal: Create a queue module that allows the main loop to
 * execute system events at a lower priority and sequentially.
 * Used primarily to organize writes to the LCD.
 */

#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#define QUEUE_SIZE 20

// The queue simply holds pointers to functions of this type 
// that can be called from the main loop
typedef void (*queue_event_cb)(void);

typedef struct event_queue_t {
    int front_index;
    int size;
    queue_event_cb queue[QUEUE_SIZE];
} event_queue_t;

void queue_init(void);
void queue_add_event(queue_event_cb cb);
void queue_process_all_events(void);
void queue_wait_for_event(void);




#endif
