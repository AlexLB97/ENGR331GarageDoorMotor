/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Goal: Module for initializing and handling input from motion detector.
 */

#include "event_queue.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "low_power.h"
#include "motion_detector.h"
#include "servo_control.h"

#include <stdbool.h>
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define MOTION_PIN 4
#define INT_DELAY_MS 5000
#define OCCUPANCY_TIMEOUT_S 30
#define INIT_TIME_S 60


/* File Scope Variables */
static timer_t input_delay_timer;
static timer_t occupancy_timer;
static bool motion_detection_active = true;
static occupancy_state_t occupancy_state = GARAGE_UNOCCUPIED;

/* Static Function Declarations */
extern void EXTI4_IRQHandler(void);
static void delay_timer_cb(void);
static void occupancy_timer_cb(void);


/* Function Definitions */

// Getter method to provide the current occupancy state to other modules
occupancy_state_t motion_detector_get_occupancy_state(void)
{
    return occupancy_state;
}

// Callback that handles the transition from unoccupied to occupied state
void transition_to_occupied_state_cb(void)
{
    motion_detector_handle_state_transition(GARAGE_OCCUPIED);
}

// Callback that handles the transition from occupied ot unoccupied state
static void transition_to_unoccupied_state_cb(void)
{
    motion_detector_handle_state_transition(GARAGE_UNOCCUPIED);
}

// Function to centralize the state transition sequences.
void motion_detector_handle_state_transition(occupancy_state_t new_state)
{
    // Update module-level state
    occupancy_state = new_state;

    switch (occupancy_state)
    {
        case GARAGE_UNOCCUPIED:
        {
            if (servo_control_get_current_state() == DOOR_STATE_CLOSED)
            {
                // Enter low power mode if the garage is closed and unoccupied
                low_power_schedule_sleep();
            }
            else
            {
                // Close the garage door
                servo_control_close_door();
                // Stop the occupancy timer 
                timer_stop_timer(&occupancy_timer);
            }
            break;
        }

        case GARAGE_OCCUPIED:
        {
            // Start the occupancy timer
            timer_start_timer(&occupancy_timer);
            break;
        }
    }
}

/**
 * @brief Handles interrupts from the motion detector
 */
void EXTI4_IRQHandler(void)
{
    if (motion_detection_active)
    {
        // Start timer and disallow interrupts to throttle back
        motion_detection_active = false;

        // Start timer that prevents high interrupt frequency from motion detector
        timer_start_timer(&input_delay_timer);

        // Reset the occupancy timer each time motion is detected
        timer_reset_timer(&occupancy_timer);

        // Handle transition to occupied state if necessary.
        queue_add_event(transition_to_occupied_state_cb);
    }

    // Clear interrupt flag
    EXTI->PR |= (1 << MOTION_PIN);
}

// Allow another interrupt from the motion detector
static void delay_timer_cb(void)
{
    motion_detection_active = true;
}

// Transition to unoccupied state when no motion is detected for extended period of time
static void occupancy_timer_cb(void)
{
    queue_add_event(transition_to_unoccupied_state_cb);
}

// Initialize motion detector module
void motion_detector_init(void)
{
    // Initialize clock for port C if not enabled
    gpio_clock_enable(RCC_AHB1ENR_GPIOCEN_Pos);
    gpio_clock_enable(RCC_AHB1ENR_GPIOEEN_Pos);

	// Set motion dectector pin to input
	gpio_pin_set_mode(GPIOC, GPIO_CREATE_MODE_MASK(MOTION_PIN, GPIO_MODE_INPUT));

	// Set PUPDR for button
	gpio_set_pupdr(GPIOC, GPIO_CREATE_MODE_MASK(MOTION_PIN, GPIO_PUPDR_NO_PULL));

    // Enable motion detector interrupt
    
    // Set ICR Register
    SYSCFG->EXTICR[1] &= ~(0x000Fu);
    SYSCFG->EXTICR[1] |= 0x02;
    EXTI->IMR |= (1 << MOTION_PIN);
    EXTI->RTSR |= (1 << MOTION_PIN);
    NVIC_SetPriority(EXTI4_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);

    // Create timer to throttling back motion sensor input a bit
    timer_create_timer(&input_delay_timer, false, INT_DELAY_MS, delay_timer_cb);
    timer_create_timer(&occupancy_timer, false, (OCCUPANCY_TIMEOUT_S * 1000), occupancy_timer_cb);
    
    // Device boots because motion was detected. Transition to occupied state immediately.
    motion_detector_handle_state_transition(GARAGE_OCCUPIED);

}
