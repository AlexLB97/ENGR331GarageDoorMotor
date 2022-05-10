/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Goal: Module for initializing and handling input from motion detector.
 */

#include "lab_gpio.h"
#include "lab_timers.h"
#include "motion_detector.h"
#include "servo_control.h"

#include <stdbool.h>
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define MOTION_PIN 4
#define INT_DELAY_MS 5000
#define OCCUPANCY_TIMEOUT_S 20
#define INIT_TIME_S 60


/* File Scope Variables */
static timer_t input_delay_timer;
static timer_t occupancy_timer;
static timer_t initialization_timer;
static bool initialized = true;
static bool motion_detection_active = true;
static occupancy_state_t occupancy_state = GARAGE_UNOCCUPIED;

/* Static Function Declarations */
extern void EXTI4_IRQHandler(void);
static void delay_timer_cb(void);
static void occupancy_timer_cb(void);
static void initialization_timer_cb(void);


/* Function Definitions */

occupancy_state_t motion_detector_get_occupancy_state(void)
{
    return occupancy_state;
}


/**
 * @brief Handles interrupts from the motion detector
 */
void EXTI4_IRQHandler(void)
{
    if (motion_detection_active)
    {
        // Set garage state as occupied
        occupancy_state = GARAGE_OCCUPIED;
        
        // Start timer and disallow interrupts to throttle back
        motion_detection_active = false;
        timer_start_timer(&input_delay_timer);

        // Reset garage inactivity timer
        timer_reset_timer(&occupancy_timer);
    }

    // Clear interrupt flag
    EXTI->PR |= (1 << MOTION_PIN);
}

static void delay_timer_cb(void)
{
    motion_detection_active = true;
    gpio_pin_clear(GPIOD, ORANGE_LED);
}


static void occupancy_timer_cb(void)
{
    // Set garage as unoccupied
    occupancy_state = GARAGE_UNOCCUPIED;

    // Close the garage door
    servo_control_close_door();
}

static void initialization_timer_cb(void)
{
    initialized = true;
}



void motion_detector_init(void)
{
    // Initialize clock for port C if not enabled
    if (!(RCC->AHB1ENR & (1 << RCC_AHB1ENR_GPIOCEN_Pos)))
    {
        gpio_clock_enable(RCC_AHB1ENR_GPIOCEN_Pos);
    }

	// Set motion dectector pin to input
	gpio_pin_set_mode(GPIOC, GPIO_CREATE_MODE_MASK(MOTION_PIN, GPIO_MODE_INPUT));

	// Set PUPDR for button
	gpio_set_pupdr(GPIOC, GPIO_CREATE_MODE_MASK(MOTION_PIN, GPIO_PUPDR_PULLUP));
    
    // Enable motion detector interrupt
    
    // Set ICR Register
    SYSCFG->EXTICR[0] &= ~(0x000Fu);
    SYSCFG->EXTICR[1] |= 0x02;
    EXTI->IMR |= (1 << MOTION_PIN);
    EXTI->RTSR |= (1 << MOTION_PIN);
    NVIC_SetPriority(EXTI4_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);

    // Create timer to throttling back motion sensor input a bit
    timer_create_timer(&input_delay_timer, false, INT_DELAY_MS, delay_timer_cb);
    timer_create_timer(&occupancy_timer, false, (OCCUPANCY_TIMEOUT_S * 1000), occupancy_timer_cb);
    timer_create_timer(&initialization_timer, false, (INIT_TIME_S * 60 * 1000), initialization_timer_cb);

    // Start init timer to prevent use of this module before it is ready

}
