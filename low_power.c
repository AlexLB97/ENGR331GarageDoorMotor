/**
 * Authors: Alex Bourdage, Sophie Woessner
 * 
 * This module manages low power entry and exit.
 */

#include "low_power.h"

#include "stm32f407xx.h"

#include "event_queue.h"
#include "lab_gpio.h"
#include "LCD.h"
#include "lcd_layout.h"
#include "motion_detector.h"


#define LOW_POWER_EXIT_PIN 0

// Static function declarations
static void low_power_enter_sleep_cb(void);


// Function to configure 
static void low_power_exit_pin_init(void)
{
	// Enable clock for port A
	gpio_clock_enable(RCC_AHB1ENR_GPIOAEN_Pos);

	// Set PA0 to input
	gpio_pin_set_mode(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_MODE_INPUT));

	// Set pulldown
	gpio_set_pupdr(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_PUPDR_PULLDOWN));
}

// Function to call sleep entry from main loop. This ensures that no interrupts are active when it is called
void low_power_schedule_sleep(void)
{
    queue_add_event(low_power_enter_sleep_cb);
}

// Function for placing the Cortex-M4 in Standby Mode for low power
static void low_power_enter_sleep_cb(void)
{
    // Only go to sleep if the garage is unoccupied
    if (motion_detector_get_occupancy_state() == GARAGE_UNOCCUPIED)
    {
        // Display sleep message on LCD
        LCD_clear_display();
        LCD_write_string("Low Power Mode", ON_WHILE_WRITING);


        // Disable all interrupts
        NVIC_DisableIRQ(SysTick_IRQn);
        NVIC_DisableIRQ(EXTI1_IRQn);
        NVIC_DisableIRQ(EXTI2_IRQn);
        NVIC_DisableIRQ(EXTI3_IRQn);
        NVIC_DisableIRQ(EXTI4_IRQn);
        NVIC_DisableIRQ(EXTI15_10_IRQn);
        NVIC_DisableIRQ(SysTick_IRQn);
        NVIC_DisableIRQ(ADC_IRQn);
        
        // Ensure there are no pending IRQs
        NVIC_ClearPendingIRQ(SysTick_IRQn);
        NVIC_ClearPendingIRQ(EXTI1_IRQn);
        NVIC_ClearPendingIRQ(EXTI2_IRQn);
        NVIC_ClearPendingIRQ(EXTI3_IRQn);
        NVIC_ClearPendingIRQ(EXTI4_IRQn);
        NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        NVIC_ClearPendingIRQ(SysTick_IRQn);
        NVIC_ClearPendingIRQ(ADC_IRQn);
        
        // Enable clock to the PWR register
        RCC->APB1ENR |= (1 << 28);
        
        // Configure to enter standby mode
        PWR->CR |= PWR_CR_PDDS_Msk;
        
        // Clear any existing wakeup flag
        PWR->CR |= PWR_CR_CWUF_Msk;
        
        // Enable the wakeup pin (PA0)
        PWR->CSR |= PWR_CSR_EWUP_Msk;
        
        // Set the enter deep sleep bit
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  
        

        // Configure interrupt for PA0 that will wake the device
        SYSCFG->EXTICR[0] &= ~(0x000Fu);
        EXTI->IMR |= (1 << USER_BTN);
        EXTI->RTSR |= (1 << USER_BTN);
        NVIC_SetPriority(EXTI0_IRQn, 0);
        NVIC_ClearPendingIRQ(EXTI0_IRQn);
        NVIC_EnableIRQ(EXTI0_IRQn);
        
        // Call wait for interrupt to enter sleep mode.
        __WFI();
    }
}

// Initialize low power module
void low_power_init(void)
{
    low_power_exit_pin_init();
}

