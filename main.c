/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 */

/* Includes */

#include <stdbool.h>

#include "adc.h"
#include "keypad.h"
#include "lab_gpio.h"
#include "lab_interrupts.h"
#include "lab_timers.h"
#include "LCD.h"
#include "lcd_layout.h"
#include "motion_detector.h"
#include "motor_control.h"
#include "servo_control.h"
#include "stm32f407xx.h"

extern void EXTI0_IRQHandler(void);

/* Preprocessor Definitions */
#define DEBOUNCE_TIMER_PERIOD_MS 200

/* File Scope Variables */
static timer_t debounce_timer;
static bool button_press_allowed = true;

/* Static Function Declarations */
static void debounce_timer_cb(void);

/* Static Functions */

// Timer callback that re-enables the button interrupt after a debounce period
static void debounce_timer_cb(void)
{
    button_press_allowed = true;
}


/**
 * @brief User button interrupt handler.
 * TODO: Replace user button with keypad to open
 */
void EXTI0_IRQHandler(void)
{
    if (button_press_allowed)
    {
        // Transition motor to next state
        door_state_t next_state = servo_control_get_next_state();

        servo_control_handle_state_transition(next_state);
        
        button_press_allowed = false;
        
        // Start timer to generate delay before re-enabling IRQ
        timer_start_timer(&debounce_timer);
    }

    // Clear the interrupt flag to allow exiting this ISR
    EXTI->PR |= 1;
}


static void board_init(void)
{
    // Enable clock for SYSCFG to allow configuration of interrupts
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;
}


/* Main function */

int main(void)
{
    // Enable prerequisites for using the functionality in this project (clocks, mostly)
    board_init();
    
    // Initialize the user button
    // user_button_init();

    // Initialize the motor control system
    motor_control_init();

    // Initialize the timers module
    timers_init();

    // Create button debounce timer
    timer_create_timer(&debounce_timer, false, DEBOUNCE_TIMER_PERIOD_MS, debounce_timer_cb);
    
    
    // Initialize the LCD. Must be done before any modules that write to the LCD
    LCD_port_init();
    
    LCD_init();
    
    keypad_init();

    adc_init();
    
    servo_init();

    motion_detector_init();

    // Enable interrupts
    interrupts_init_interrupts();

    while(1);
}
