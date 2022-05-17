/**
 * Authors: Alex Bourdage and Sophie Woessner
 * 
 * Goal: Create a queue module that allows the main loop to
 * execute system events at a lower priority and sequentially.
 * Used primarily to organize writes to the LCD.
 */

#include "event_queue.h"

#include <stdbool.h>

#include "lab_timers.h"

/* File Scope Variables */
static event_queue_t event_queue;


/* Static Function Definitions */
static bool queue_process_next_event(void);


// Allocates space and performs initialization for the queue.
void queue_init(void)
{
    event_queue.front_index = 0;
    event_queue.size = 0;
}

// Adds an event to the next open spot in the queue, wrapping back to the front after
// exceeding the length of the queue
void queue_add_event(queue_event_cb cb)
{
    if (event_queue.size < QUEUE_SIZE)
    {
        // Put the callback in the next available spot
        int next_index = (event_queue.front_index + event_queue.size) % QUEUE_SIZE;
        event_queue.queue[next_index] = cb;
        event_queue.size++;
    }
}

/**
 * @brief Function that processes the next event in the queue and removes it.
 * 
 * @param[out] event_processed  True if an event was processed, false otherwise
 */
static bool queue_process_next_event(void)
{
    bool event_processed;

    if (event_queue.size == 0)
    {
        event_processed = false;
    } 
    else
    {
        // Execute the next event in the queue
        event_queue.queue[event_queue.front_index]();
        event_queue.size--;
        event_queue.front_index = (event_queue.front_index + 1) % QUEUE_SIZE;
        event_processed = true;
    }

    return event_processed;
}

void queue_process_all_events(void)
{
    // Calls process next event until it returns false, indicating that the queue is empty
    while(queue_process_next_event());
}

/**
 * @brief Function that blocks the main loop until an event is in the queue
 */
void queue_wait_for_event(void)
{
    while(event_queue.size == 0);
}
