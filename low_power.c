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
#include "motion_detector.h"


#define LOW_POWER_EXIT_PIN 0

// Static function declarations
static void low_power_enter_sleep_cb(void);

static void low_power_exit_pin_init(void)
{
	// Enable clock for port A
	gpio_clock_enable(RCC_AHB1ENR_GPIOAEN_Pos);

	// Set button to input
	gpio_pin_set_mode(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_MODE_INPUT));

	// Set PUPDR for button
	gpio_set_pupdr(GPIOA, GPIO_CREATE_MODE_MASK(USER_BTN, GPIO_PUPDR_PULLDOWN));
}

void low_power_schedule_sleep(void)
{
    queue_add_event(low_power_enter_sleep_cb);
}

static void low_power_enter_sleep_cb(void)
{
    // Only go to sleep if the garage in unoccupied
    if (motion_detector_get_occupancy_state() == GARAGE_UNOCCUPIED)
    {
        LCD_clear_display();

        // Disable all interrupts
        NVIC_DisableIRQ(SysTick_IRQn);
        NVIC_DisableIRQ(EXTI1_IRQn);
        NVIC_DisableIRQ(EXTI2_IRQn);
        NVIC_DisableIRQ(EXTI3_IRQn);
        NVIC_DisableIRQ(EXTI4_IRQn);
        NVIC_DisableIRQ(EXTI15_10_IRQn);
        NVIC_DisableIRQ(SysTick_IRQn);
        NVIC_DisableIRQ(ADC_IRQn);
        
        NVIC_ClearPendingIRQ(SysTick_IRQn);
        NVIC_ClearPendingIRQ(EXTI1_IRQn);
        NVIC_ClearPendingIRQ(EXTI2_IRQn);
        NVIC_ClearPendingIRQ(EXTI3_IRQn);
        NVIC_ClearPendingIRQ(EXTI4_IRQn);
        NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
        NVIC_ClearPendingIRQ(SysTick_IRQn);
        NVIC_ClearPendingIRQ(ADC_IRQn);
        
        RCC->APB1ENR |= (1 << 28);
        
        // Clear wakeup flag
        PWR->CR |= (1 << 2);
        PWR->CR |= (1 << 1);
        PWR->CSR |= (1 << 8);
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  
        

        // Set ICR Register
        SYSCFG->EXTICR[0] &= ~(0x000Fu);
        EXTI->IMR |= (1 << USER_BTN);
        EXTI->RTSR |= (1 << USER_BTN);
        NVIC_SetPriority(EXTI0_IRQn, 0);
        NVIC_ClearPendingIRQ(EXTI0_IRQn);
        NVIC_EnableIRQ(EXTI0_IRQn);
        
        __WFI();
    }
}


void low_power_init(void)
{
    low_power_exit_pin_init();
}

