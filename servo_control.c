/**
 * Alex Bourdage
 * Sophie Woessner
 * 
 * Module for controlling the servo motor
 */

#include "servo_control.h"

#include <stdint.h>
#include <string.h>

#include "global_config_info.h"
#include "LCD.h"
#include "lcd_layout.h"
#include "lab_gpio.h"
#include "lab_timers.h"
#include "motion_detector.h"
#include "stm32f407xx.h"
#include "event_queue.h"

/* Preprocessor Definitions */
#define SERVO_PIN 8
#define SERVO_AF_NUM 2
#define SERVO_PWM_FREQ_HZ 50
#define PSC_FREQ_HZ 180000
#define POSITION_0_COMPARE_VAL 88
#define MAX_ANGLE 180
#define OPEN_ANGLE 90
#define CLOSED_ANGLE 0
#define DOOR_OPEN_TIME_S 5
#define STEP_SIZE_DEGREES 1

/* File Scope Variables */
static timer_t servo_timer;
static uint32_t servo_angle = 0;
static door_state_t current_state = DOOR_STATE_CLOSING;
static door_state_t previous_state = DOOR_STATE_CLOSING;

static char state_strings[5][10] = {"OPENING", "CLOSING", "STOPPED", "CLOSED", "OPEN"};
static char clear_string[] = "        ";



/* Static Function Declarations */
static void servo_timer_cb(void);
static void servo_set_angle(uint32_t angle);
static void servo_pwm_init(void);
static void clear_door_status_region(void);

/* Function Definitions */

static void clear_door_status_region(void)
{
    LCD_write_string_at_addr(clear_string, ON_WHILE_WRITING, FIRST_LINE_STRT_ADDR, (int)strlen(clear_string));
}

static void update_status_cb(void)
{
    clear_door_status_region();
    LCD_write_string_at_addr(state_strings[current_state], ON_WHILE_WRITING, FIRST_LINE_STRT_ADDR, (int)strlen(state_strings[current_state]));
}

void servo_control_close_door(void)
{
    servo_control_handle_state_transition(DOOR_STATE_CLOSING);
}

void servo_control_open_door(void)
{
    servo_control_handle_state_transition(DOOR_STATE_OPENING);
}

void servo_control_stop_door(void)
{
    servo_control_handle_state_transition(DOOR_STATE_STOPPED);
}

static void servo_timer_cb(void)
{
    switch(current_state)
    {
        case DOOR_STATE_OPEN:
        case DOOR_STATE_CLOSED:
        case DOOR_STATE_STOPPED:
            // Do nothing in these states
            break;
        
        case DOOR_STATE_OPENING:
        {
            if (servo_angle >= OPEN_ANGLE)
            {
                servo_control_handle_state_transition(DOOR_STATE_OPEN);
            }
            else
            {
                servo_angle += STEP_SIZE_DEGREES;
                servo_set_angle(servo_angle);
            }
            break;
        }

        case DOOR_STATE_CLOSING:
        {
            if (servo_angle <= CLOSED_ANGLE)
            {
                servo_control_handle_state_transition(DOOR_STATE_CLOSED);
            }
            else
            {
                servo_angle -= STEP_SIZE_DEGREES;
                servo_set_angle(servo_angle);
            }
            break;
        }
    }
}


static void servo_pwm_init(void)
{
    // Enable clock to GPIOD
    gpio_clock_enable(RCC_AHB1ENR_GPIOCEN_Pos);

    // Enable clock to TIM4
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Put pin PD13 in alternate function mode
    gpio_pin_set_mode(GPIOC, GPIO_CREATE_MODE_MASK(SERVO_PIN, GPIO_MODE_AF));

    // Connect pin in AFR
    GPIOC->AFR[SERVO_PIN / 8] |= (SERVO_AF_NUM << ((SERVO_PIN % 8) * 4));

    // Set timer 4 channel 4 as an output
    TIM3->CCMR2 &= ~(0x2u << 0);
    
    // Set polarity to active high
    TIM3->CCER &= ~(0x1u << 9);
    
    // Set PWM to mode 1
    TIM3->CCMR2 &= ~(0x7u << 4);
    TIM3->CCMR2 |= (0x6u << 4);
    
    // Set the PSC such that we have 1 degree resolution (180000 Hz)
    TIM3->PSC = (SYSCLK_FREQ_HZ / PSC_FREQ_HZ);
    
    // Set the ARR to set the period of the PWM
    TIM3->ARR = ((SYSCLK_FREQ_HZ / (TIM3->PSC + 1)) / SERVO_PWM_FREQ_HZ) - 1;

    TIM3->CCR3 = POSITION_0_COMPARE_VAL; // Set initial duty cycle to 1ms pulse length for position 0
    
    // Set output preload enable bit
    TIM3->CCMR2 |= (1 << 3);

    // Set the auto-preload enable bit
    TIM3->CR1 |= (1 << 7);
    
    // Clear the alignment and direction bits.
    TIM3->CR1 &= ~(0x7u << 4);
    
    // Enable CC3 compare
    TIM3->CCER |= (1 << 8);
    
    // Enable the counter
    TIM3->CR1 |= TIM_CR1_CEN;

    // Set the door to its initial closed position
    servo_set_angle(CLOSED_ANGLE);
}

static void servo_set_angle(uint32_t angle)
{
    // Each tick has a resolution of 1 degree due to PSC value
    // 0 degrees is at 180 ticks and 180 degrees is at 360 ticks
    // 180 degrees is maximum movement

    if (angle > MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }

    TIM3->CCR3 = POSITION_0_COMPARE_VAL + (angle * 2);
}

door_state_t servo_control_get_current_state(void)
{
    return current_state;
}

/**
 * @brief Function that manages the sequence of state for the door and returns the next state based
 * on current and the event that occurred.
 *
 * @param [in] current_state  The current state of the door.
 * @param [in] event          Event that occurred in the system
 * @param [out] state         The next state of the system
 */
door_state_t servo_control_get_next_state(void)
{
    // The sequence of events should be STOPPED->OPENING->STOPPED->CLOSING->REPEAT
    switch (current_state)
    {
        case DOOR_STATE_OPEN:
            return DOOR_STATE_CLOSING;
        
        case DOOR_STATE_CLOSED:
            return DOOR_STATE_OPENING;

        case DOOR_STATE_STOPPED:
            if (previous_state == DOOR_STATE_CLOSING)
            {
                return DOOR_STATE_OPENING;
            }
            else
            {
                return DOOR_STATE_CLOSING;
            }
        
        case DOOR_STATE_CLOSING:
        case DOOR_STATE_OPENING:
            return DOOR_STATE_STOPPED;
    }
}

void servo_control_handle_state_transition(door_state_t next_state)
{
    if (next_state != current_state)
    {
        previous_state = current_state;
        current_state = next_state;
        queue_add_event(update_status_cb);
    }
}

/**
 * @brief Function for initializing the pins and PWWM associated with the servo
 * motor.
 */
void servo_init(void)
{
    uint32_t servo_timer_period_ms = DOOR_OPEN_TIME_S * 1000 / (OPEN_ANGLE - CLOSED_ANGLE) * STEP_SIZE_DEGREES;
    servo_pwm_init();
    timer6_delay(100);
    timer_create_timer(&servo_timer, true, servo_timer_period_ms, servo_timer_cb);
    timer_start_timer(&servo_timer);
    servo_control_handle_state_transition(DOOR_STATE_CLOSED);
}
