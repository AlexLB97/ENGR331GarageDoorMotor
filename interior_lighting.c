/**
 * Authors: Alex Bourdage, Sophie Woessner
 * 
 * This module is responsible for keeping track of the interior lighting conditions and toggling
 * the interior light on and off accordingly.
 */

#include "interior_lighting.h"

#include "adc.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "motion_detector.h"
#include "event_queue.h"


// High level design:

// ADC module automatically updates the light level after each reading
// Motion detector module updates occupancy state when it changes
// Once per second, this module will look at those states and determine
// the state that the light should be in.

/* Preprocessor Definitions */
#define LIGHTING_TIMER_PERIOD_MS 1000
#define LIGHT_ON_THRESHOLD 0.2
#define LIGHT_OFF_HYSTERESIS 0.1
#define LIGHT_OFF_THRESHOLD (LIGHT_ON_THRESHOLD + LIGHT_OFF_HYSTERESIS)
#define WHITE_LED 10

/* Typedefs */

// Type for keeping track of LED state in a readable way
typedef enum {
    LIGHT_ON = 0,
    LIGHT_OFF
} interior_lighting_state_t;

/* Static Function Declarations */
static void periodic_lighting_cb(void);\
static void update_interior_lighting_cb(void);
static void change_light_state(interior_lighting_state_t state);

/* File scope variable declarations */
static timer_t lighting_timer;

/*Function Definitions */

// Function for transitioning the light between on and off states.
static void change_light_state(interior_lighting_state_t state)
{
    if (state == LIGHT_ON)
    {
        gpio_pin_set(GPIOE, WHITE_LED);
    }
    else
    {
        gpio_pin_clear(GPIOE, WHITE_LED);
    }
}

// Callback for the timer that periodically updates the state of the LED
static void periodic_lighting_cb(void)
{
    queue_add_event(update_interior_lighting_cb);
}

// Callback from executed from the main loop that updated the state of the LED based on
// current occupancy and lighting conditions.
static void update_interior_lighting_cb(void)
{
    if (motion_detector_get_occupancy_state() == GARAGE_OCCUPIED)
    {
        double light_level = adc_get_light_level();
        if (light_level < LIGHT_ON_THRESHOLD)
        {
            change_light_state(LIGHT_ON);
        }
        else if (light_level > LIGHT_OFF_THRESHOLD)
        {
            change_light_state(LIGHT_OFF);
        }
    }
    else
    {
        change_light_state(LIGHT_OFF);
    }
}

// Function to initialize the module by creating timers and configuring GPIO.
void interior_lighting_init(void)
{
    timer_create_timer(&lighting_timer, true, LIGHTING_TIMER_PERIOD_MS, periodic_lighting_cb);
    timer_start_timer(&lighting_timer);

    // Initialize interior LED
    gpio_clock_enable(RCC_AHB1ENR_GPIOEEN_Pos);
    gpio_pin_set_mode(GPIOE, GPIO_CREATE_MODE_MASK(WHITE_LED, GPIO_MODE_OUTPUT));
    gpio_set_pupdr(GPIOE, GPIO_CREATE_PUPDR_MASK(WHITE_LED, GPIO_PUPDR_NO_PULL));
}

