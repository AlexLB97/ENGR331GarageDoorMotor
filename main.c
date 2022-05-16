/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 */

/* Includes */

#include <stdbool.h>

#include "adc.h"
#include "break_beam.h"
#include "interior_lighting.h"
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
#include "event_queue.h"

/* Preprocessor Definitions */

/* File Scope Variables */

/* Static Function Declarations */

/* Static Functions */


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

    // Initialize the motor control system
    motor_control_init();

    // Initialize the timers module
    timers_init();    
    
    // Initialize the LCD. Must be done before any modules that write to the LCD
    LCD_port_init();
    
    LCD_init();
    
    keypad_init();

    adc_init();

    break_beam_init();
    
    servo_init();

    motion_detector_init();

    interior_lighting_init();

    motor_control_init();

    // Enable interrupts
    interrupts_init_interrupts();

    while(1)
    {
        queue_wait_for_event();
        queue_process_all_events();
    }
}
