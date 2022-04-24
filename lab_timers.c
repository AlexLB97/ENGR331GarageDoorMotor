/* Authors: Alex Bourdage, Sophie Woessner
 * Description - Module for initializing configuring timers.
 */
#include "lab_timers.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f407xx.h"

#define UINT16_MAX 65535

static void check_for_expired_timers(void);
static void remove_timer_from_list(timer_t *pTimer);
static void shift_timers_left(int start_index);
extern void SysTick_Handler(void);


/**
 * Initializes TIM6 to a set period and enables the interrupt.
 */
void timers_init_timer(TIM_TypeDef * timer, uint32_t apb1enr_bus_position, uint32_t period_ms)
{
    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t divisor = 0;

    // Calculate ARR and prescaler for requested period
    divisor = (HSI_SPEED_HZ / 1000) * period_ms;
    prescaler = (divisor - 1) < UINT16_MAX ? (divisor - 1) : (UINT16_MAX);
    if (prescaler == UINT16_MAX)
    {
        arr = (divisor / prescaler) - 1;
    }

    // enable APB1 bus clock
	RCC->APB1ENR|= apb1enr_bus_position;
	//TIM6 prescaler set at default to 0 for now
	timer->PSC = prescaler; // prescalar
	timer->ARR = arr;  //auto reload register 
	timer->CNT=0;   //clear counter register
	timer->CR1|=TIM_CR1_CEN; // Enable the timer
    timer->DIER |= TIM_DIER_UIE; // Enable the timer interrupt event
    timer->SR = 0; // Clear the interrupt flag
}

void timers_change_period(TIM_TypeDef *timer, uint32_t period_ms)
{
    uint32_t prescaler = 0;
    uint32_t arr = 0;
    uint32_t divisor = 0;

    // Calculate new prescaler and auto-reload value
    divisor = (HSI_SPEED_HZ / 1000) * period_ms;
    prescaler = (divisor - 1) < UINT16_MAX ? (divisor - 1) : (UINT16_MAX);
    if (prescaler == UINT16_MAX)
    {
        arr = (divisor / prescaler) - 1;
    }
    
    // Update and reset timer with new values 
    timer->PSC = prescaler; // prescalar
	timer->ARR = arr;  //auto reload register 
    timer->CNT = 0; // reset count to 0
    timer->SR = 0; // Make sure interrupt flag is clear
}

/**
 * Start of code for SysTick based timer library. The goal
 * of this library is to be able to set up one-shot or repeating
 * timers that execute callbacks without blocking the main thread or using
 * other timer resources. It will maintain a list of active timers and their associated information.
 * 
 * NOTE: Memory space for timers should be allocated in the module that uses the timer at compile time.
 * 
 * Information needed for each timer:
 * Number of systick iterations until expiration
 * Callback on expiration
 * Duration
 * One shot or repeating
 * Timer state (active or not)
 */

/**
 * Type declarations for timer module
 */

#define MAX_TIMERS 10


/**
 * VARIABLES FOR SYSTICK TIMER MODULE
 */

static uint32_t ticks = 0;

static timer_t *timer_list[MAX_TIMERS];

static int num_active_timers = 0;

// Use an array to hold a maximum number of timers here in static memory.
// 


/**
 * Function to initialize the systick interrupt, upon which the timer module will be based,
 * to a specific interval. This function also needs to initialize the array that will
 * be used to store the list of active timers.
 */
void timers_init()
{
    // Configure Systick for 1ms interval
    SysTick->LOAD = 16000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 7;
}

/**
 * This function executes each time the systick interrupt fires, checks each timer for expiration,
 * and executes the callback if it was expired. Then it either deletes the timer if it was a one shot
 * or calculates the new end time if it repeats.
 */
void SysTick_Handler(void)
{
    ticks++;
    check_for_expired_timers();
}

static void check_for_expired_timers(void)
{
    for (int i = 0; i < num_active_timers; i++)
    {
        if (timer_list[i]->expiration_ticks < ticks)
        {
            timer_list[i]->cb();
            if (timer_list[i]->repeating)
            {
                timer_list[i]->expiration_ticks = ticks + timer_list[i]->duration_ms;
            }
            else
            {
                timer_stop_timer(timer_list[i]);
                i--;
            }
        }
    }
}



/**
 * Function to create a new timer
 */
void timer_create_timer(timer_t *pTimer, bool repeating, uint32_t period_ms, timer_callback_t cb)
{
    pTimer->repeating = repeating;
    pTimer->duration_ms = period_ms;
    pTimer->cb = cb;
}


/**
 * Function to start a timer
 */

void timer_start_timer(timer_t *pTimer)
{
    pTimer->expiration_ticks = ticks + pTimer->duration_ms;
    pTimer->timer_active = true;
    timer_list[num_active_timers] = pTimer;
    num_active_timers++;
}

/**
 * Function to get current tick count
 */
uint32_t timer_get_time_millis(void)
{
    return ticks;
}

/**
 * Function to reset a timer.
 */
void timer_reset_timer(timer_t *pTimer)
{
    timer_stop_timer(pTimer);
    timer_start_timer(pTimer);
}

/**
 * Function to stop a timer
 */

void timer_stop_timer(timer_t *pTimer)
{
    pTimer->timer_active = false;
    remove_timer_from_list(pTimer);
}

static void remove_timer_from_list(timer_t *pTimer)
{
    for (int i = 0; i < num_active_timers; i++)
    {
        if (timer_list[i] == pTimer)
        {
            shift_timers_left(i);
            num_active_timers--;
        }
    }
}

static void shift_timers_left(int start_index)
{
    for (int i = start_index; i < num_active_timers - 1; i++)
    {
        timer_list[i] = timer_list[i + 1];
    }
}

/**
 * Function to change timer period
 */
void timer_change_timer_period(timer_t *pTimer, uint32_t new_period_ms)
{
    pTimer->duration_ms = new_period_ms;
    timer_stop_timer(pTimer);
    timer_start_timer(pTimer);
}

bool timer_is_timer_active(timer_t *pTimer)
{
    return pTimer->timer_active;
}

/**
 * Delay function based on the timer module.
 * 
 * NOTE: This delay function will not work if called from within an IRQ with a higher
 * priority than SysTick
 */

void delay(uint32_t ms)
{
    uint32_t timeout = ticks + ms;

    while(ticks < timeout);
}
