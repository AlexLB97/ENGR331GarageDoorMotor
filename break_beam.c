/**
 * Authors: ALex Bourdage, Sophie Woessner
 * 
 * Goal: Module for handling input from the break beam sensor.
 */


#include "break_beam.h"

#include <stdbool.h>

#include "lab_gpio.h"
#include "servo_control.h"
#include "stm32f407xx.h"

/* Preprocessor Definitions */
#define BREAK_BEAM_PIN 12

/* File Scope Variables */
static bool break_beam_enabled = false;

/* Static Function Declarations */
extern void EXTI15_10_IRQHandler(void);

/* Function Definitions */

void enable_break_beam(void)
{
    break_beam_enabled = true;
}

void disable_break_beam(void)
{
    break_beam_enabled = false;
}

/**
 * @brief Break beam interrupt handler.
 */
void EXTI15_10_IRQHandler(void)
{
    if (break_beam_enabled)
    {
        // Open the door again if beam was broken
        servo_control_handle_state_transition(DOOR_STATE_OPENING);
        
        // Break beam is now disabled. Will be reactivated again when door starts to close.
        break_beam_enabled = false;
    }

    // Clear the interrupt flag to allow exiting this ISR
    EXTI->PR |= (1 << BREAK_BEAM_PIN);
}

void break_beam_init(void)
{
    // Enable clock for PE12
    gpio_clock_enable(RCC_AHB1ENR_GPIOEEN_Pos);

    // Configure as pin as an input with a pullup
    gpio_pin_set_mode(GPIOE, GPIO_CREATE_MODE_MASK(BREAK_BEAM_PIN, GPIO_MODE_INPUT));
    gpio_set_pupdr(GPIOE, GPIO_CREATE_PUPDR_MASK(BREAK_BEAM_PIN, GPIO_PUPDR_PULLUP));

    // Configure interrupt for break beam
    SYSCFG->EXTICR[3] &= ~(0x000Fu);
    SYSCFG->EXTICR[3] |= (0x0004u);
    EXTI->IMR |= (1 << BREAK_BEAM_PIN);
    EXTI->RTSR |= (1 << BREAK_BEAM_PIN);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}
